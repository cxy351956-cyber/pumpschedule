from Epanet2_2_package import *
import numpy as np
import math
import wntr
import gurobipy as gp
import pandas as pd


def pipe_characteristic(R, q):
    dh = R * np.sign(q) * ((abs(q) / 1000) ** 1.852)
    return dh

def pump_characteristic(pump, q):
    A = pump['A']
    B = pump['B']
    C = pump['C']
    h = A - B * (q ** C)
    return h

def max_pwl_error_pipe(R, q_pts):
       alpha = 1.852 * 0.852 / (1000.0 ** 1.852)
    max_err = 0.0
    for i in range(len(q_pts) - 1):
        qa, qb = q_pts[i], q_pts[i + 1]
        if qa <= 0 <= qb:
            q_abs_min = 1e-6
        else:
            q_abs_min = max(min(abs(qa), abs(qb)), 1e-6)
        omega = abs(R) * alpha * (q_abs_min ** (-0.148))
        err = (1.0 / 8.0) * omega * ((qb - qa) ** 2)
        if err > max_err:
            max_err = err
    return max_err


def max_pwl_error_pump(func, q_pts, n_check=500):
    max_err = 0.0
    for i in range(len(q_pts) - 1):
        qa, qb = q_pts[i], q_pts[i + 1]
        ha, hb = func(qa), func(qb)
        for qs in np.linspace(qa, qb, n_check + 2)[1:-1]:
            t = (qs - qa) / (qb - qa)
            err = abs(func(qs) - (ha + t * (hb - ha)))
            if err > max_err:
                max_err = err
    return max_err

def adaptive_pwl_breakpoints(R, q_min, q_max, epsilon, max_pow=5):
    def func(q): return pipe_characteristic(R, q)
    for k in range(0, max_pow + 1):
        n_seg = 2 ** k
        q_pts = list(np.linspace(q_min, q_max, n_seg + 1))
        if max_pwl_error_pipe(R, q_pts) <= epsilon:
            break
    return q_pts, [func(q) for q in q_pts], n_seg


def adaptive_pwl_breakpoints_pump(
            func, q_min, q_max, epsilon, max_pow=5, n_check=500):
        for k in range(0, max_pow + 1):
            n_seg = 2 ** k
            q_pts = list(np.linspace(q_min, q_max, n_seg + 1))
            if max_pwl_error_pump(func, q_pts, n_check) <= epsilon:
                break
        return q_pts, [func(q) for q in q_pts], n_seg


def add_pwl_sos2_constraints(
        model, steps, link_set, q_var, h_var, lin_q, lin_h, link_segs, prefix):

    lamda_vars = {}
    x_vars = {}
    for link in link_set:
        n_pts = link_segs[link]
        n_seg = n_pts - 1
        n_bits = math.ceil(math.log2(max(n_seg, 1)))
        for t in range(steps):
            for seg in range(n_pts):
                lamda_vars[t, link, seg] = model.addVar(
                    lb=0, ub=1, vtype=gp.GRB.CONTINUOUS,
                    name=f'lamda_{prefix}_{t}_{link}_{seg}'
                )
            for bit in range(n_bits):
                x_vars[t, link, bit] = model.addVar(
                    vtype=gp.GRB.BINARY,
                    name=f'x_{prefix}_{t}_{link}_{bit}'
                )
    model.update()
    for link in link_set:
        n_pts = link_segs[link]
        n_seg = n_pts - 1
        n_bits = math.ceil(math.log2(max(n_seg, 1)))
        for t in range(steps):
            lam = [lamda_vars[t, link, seg] for seg in range(n_pts)]
            model.addConstr(gp.quicksum(lam) == 1,
                            name=f'pwl_sum_{prefix}_{t}_{link}')
            for bit in range(n_bits):
                x_b = x_vars[t, link, bit]
                lam_bit1 = []
                lam_bit0 = []
                for seg in range(n_pts):
                    adj_segs = []
                if seg > 0:
                    adj_segs.append(seg - 1)
                if seg < n_seg:
                    adj_segs.append(seg)
                bits_of_adj = [(k >> bit) & 1 for k in adj_segs]
                if all(b == 1 for b in bits_of_adj):
                    lam_bit1.append(lam[seg])
                elif all(b == 0 for b in bits_of_adj):
                    lam_bit0.append(lam[seg])
                if lam_bit1:
                    model.addConstr(gp.quicksum(lam_bit1) <= x_b,
                                    name=f'pwl_bit1_{prefix}_{t}_{link}_{bit}')
                if lam_bit0:
                    model.addConstr(gp.quicksum(lam_bit0) <= 1 - x_b,
                                    name=f'pwl_bit0_{prefix}_{t}_{link}_{bit}')
            model.addConstr(
                q_var[t, link] == gp.quicksum(
                    lam[seg] * lin_q[link][seg] for seg in range(n_pts)),
                name=f'pwl_q_{prefix}_{t}_{link}'
            )
            model.addConstr(
                h_var[t, link] == gp.quicksum(
                    lam[seg] * lin_h[link][seg] for seg in range(n_pts)),
                name=f'pwl_h_{prefix}_{t}_{link}'
            )
    return lamda_vars, x_vars


def run_gurobi(inp_file, max_switch):
    epsilon_pipe = 0.5
    epsilon_pump = 0.5
    p_min = 40
    p_max = 100
    errcode = enOpen(inp_file, '1.rtp', '')
    jun_node = set()
    res_node = set()
    tank_node = set()
    jun_ele = {}
    res_head = {}
    tank_ele = {}
    tank_s = {}
    tank_minl = {}
    tank_maxl = {}
    tank_init = {}
    pressurenode = []
    [errcode, n_nodes] = enGetcount(0)
    for i in range(n_nodes):
        node_id = i + 1
        [errcode, type] = enGetnodeType(node_id)
        if type == 0:
            [errcode, elevation] = enGetnodevalue(node_id, 0)
            [errcode, demand] = enGetnodevalue(node_id, 1)
            jun_node.add(node_id)
            jun_ele[node_id] = elevation
            if demand != 0:
                pressurenode.append(node_id)
        elif type == 1:
            [errcode, head] = enGetnodevalue(node_id, 28)
            res_node.add(node_id)
            res_head[node_id] = head
        elif type == 2:
            [errcode, ele] = enGetnodevalue(node_id, 0)
            tank_node.add(node_id)
            tank_ele[node_id] = ele
            [errcode, initlevel] = enGetnodevalue(node_id, 8)
            tank_init[node_id] = initlevel
    wn = wntr.network.WaterNetworkModel(inp_file)
    tanks_name = wn.tank_name_list
    for tank_name in tanks_name:
        tank = wn.get_node(tank_name)
        [errcode, tank_index] = enGetnodeIndex(tank_name)
        tank_s[tank_index] = (tank.diameter ** 2) * math.pi / 4
        tank_minl[tank_index] = tank.min_level
        tank_maxl[tank_index] = tank.max_level
    alpha = 10.67
    e1 = 1.852
    e2 = 4.87
    pipe_link = set()
    pump_link = set()
    valve_link = set()
    tank_links = set()
    pipe_lengths = {}
    pipe_diameters = {}
    pipe_hw_coeffs = {}
    valve_hw_coeffs = {}
    tankpipe_hw = {}
    pipe_start = {}
    pipe_end = {}
    pump_start = {}
    pump_end = {}
    pump_id = {}
    valve_start = {}
    valve_end = {}
    tank_start = {}
    tank_end = {}
    A_fwd = {}
    A_bwd = {}
    [errcode, no_link] = enGetcount(2)
    for i in range(no_link):
        link_id = i + 1
        [errcode, link_type] = enGetlinkType(link_id)
        [errcode, start_node, end_node] = enGetlinkNodes(link_id)
        is_tank_connected = (start_node in tank_node or end_node in tank_node)
        A_fwd[link_id] = 1
        A_bwd[link_id] = 1
        [errcode, diameter] = enGetlinkvalue(link_id, 0)
        [errcode, length] = enGetlinkvalue(link_id, 1)
        [errcode, roughness] = enGetlinkvalue(link_id, 2)
        if link_type == 2:
            pump_link.add(link_id)
            pump_start[link_id] = start_node
            pump_end[link_id] = end_node
            [errcode, id] = enGetlinkID(link_id)
            pump_id[link_id] = id
            continue
        hw_coeff = (alpha * length) / ((roughness ** e1)
                                       * ((diameter / 1000) ** e2))
        if is_tank_connected:
            tank_links.add(link_id)
            tankpipe_hw[link_id] = hw_coeff
            tank_start[link_id] = start_node
            tank_end[link_id] = end_node
            if link_type == 0:
                A_fwd[link_id] = 1
                A_bwd[link_id] = 0
            continue
        if link_type == 1:
            pipe_link.add(link_id)
            pipe_lengths[link_id] = length
            pipe_diameters[link_id] = diameter
            pipe_hw_coeffs[link_id] = hw_coeff
            pipe_start[link_id] = start_node
            pipe_end[link_id] = end_node
        elif link_type == 0:
            valve_link.add(link_id)
            valve_hw_coeffs[link_id] = hw_coeff
            valve_start[link_id] = start_node
            valve_end[link_id] = end_node
    demand = {}
    for junc in jun_node:
        [errcode, basedemand] = enGetnodevalue(junc, 1)
        [errcode, demand_pattern] = enGetnodevalue(junc, 2)
        demand[junc] = []
        for t in range(steps):
            [errcode, multiplier] = enGetpatternvalue(
                int(demand_pattern), t + 1)
            demand[junc].append(multiplier * basedemand)
    pipe_min = {}
    pipe_max = {}
    pump_min = {}
    pump_max = {}
    tank_min = {}
    tank_max = {}
    wn.options.time.duration = 24 * 3600
    wn.options.time.hydraulic_timestep = 3600
    sim = wntr.sim.EpanetSimulator(wn)
    results = sim.run_sim()
    flowrate = results.link['flowrate']
    for name in flowrate.columns:
        [errcode, link_index] = enGetlinkIndex(name)
        if link_index in pipe_link:
            pipe_min[link_index] = flowrate[name].min()
            pipe_max[link_index] = flowrate[name].max()
        if link_index in pump_link:
            pump_min[link_index] = flowrate[name].min()
            pump_max[link_index] = flowrate[name].max()
        if link_index in tank_links:
            tank_min[link_index] = flowrate[name].min()
            tank_max[link_index] = flowrate[name].max()
    pump_co = {
        16: {'A': 100.00, 'B': 0.0001137, 'C': 2.38},
        17: {'A': 100.00, 'B': 0.0001137, 'C': 2.38},
        18: {'A': 120.00, 'B': 0.008569, 'C': 1.9}
    }
    pump_q_ranges = {16: (0, 300), 17: (0, 300), 18: (0, 150)}
    pressure_head = results.node['pressure']
    node_head_sim = {}
    for name in pressure_head.columns:
        try:
            [errcode, nidx] = enGetnodeIndex(name)
        except Exception:
            continue
        if nidx in jun_node:
            elev = jun_ele[nidx]
            node_head_sim[nidx] = pressure_head[name].values + elev
        elif nidx in tank_node:
            elev = tank_ele[nidx]
            node_head_sim[nidx] = pressure_head[name].values + elev
    for nidx in res_node:
        node_head_sim[nidx] = np.full(steps, res_head[nidx])
    flowrate_sim = results.link['flowrate']

    def get_h_sim(link_id, t):
        try:
            [errcode, lname] = enGetlinkID(link_id)
            q_ls = flowrate_sim[lname].iloc[t] * 1000.0
            if link_id in pipe_hw_coeffs:
                return pipe_characteristic(pipe_hw_coeffs[link_id], q_ls)
            elif link_id in valve_hw_coeffs:
                return pipe_characteristic(valve_hw_coeffs[link_id], q_ls)
            elif link_id in tankpipe_hw:
                return pipe_characteristic(tankpipe_hw[link_id], q_ls)
        except Exception:
            pass
        return 0.0
    epsilon_bigM = 300
    M_DEFAULT = 100000
    pump_bigM = {}
    for m in pump_link:
        i_node = pump_start[m]
        j_node = pump_end[m]
        [errcode, lname] = enGetlinkID(m)
        max_diff = 0.0
        for t in range(steps):
            hi = node_head_sim.get(i_node, np.zeros(steps))[t]
            hj = node_head_sim.get(j_node, np.zeros(steps))[t]
            q_ls = flowrate_sim[lname].iloc[t] * 1000.0
            co = pump_co.get(m, None)
            h_p = pump_characteristic(co, max(q_ls, 0.0)) if co else 0.0
            val = abs(hi - hj + h_p)
            if val > max_diff:
                max_diff = val
        pump_bigM[m] = max_diff + epsilon_bigM

    valve_bigM = {}
    for v in valve_link:
        i_node = valve_start[v]
        j_node = valve_end[v]
        [errcode, lname] = enGetlinkID(v)
        max_diff = 0.0
        for t in range(steps):
            hi = node_head_sim.get(i_node, np.zeros(steps))[t]
            hj = node_head_sim.get(j_node, np.zeros(steps))[t]
            h_v = get_h_sim(v, t)
            val = abs(hi - hj - h_v)
            if val > max_diff:
                max_diff = val
        valve_bigM[v] = max_diff + epsilon_bigM

    lin_pipe_q = {}
    lin_pipe_h = {}
    pipe_n_pts = {}
    for pipe in pipe_link:
        q_lo = min(pipe_min[pipe], -200)
        q_hi = max(pipe_max[pipe], 240)
        R = pipe_hw_coeffs[pipe]
        bq, bh, n_seg = adaptive_pwl_breakpoints(
            R, q_lo, q_hi, epsilon=epsilon_pipe)
        lin_pipe_q[pipe] = bq
        lin_pipe_h[pipe] = bh
        pipe_n_pts[pipe] = n_seg + 1

    lin_valve_q = {}
    lin_valve_h = {}
    valve_n_pts = {}
    for v in valve_link:
        q_lo = 0.0
        q_hi = max(pipe_max.get(v, 160), 160)
        R = valve_hw_coeffs[v]
        bq, bh, n_seg = adaptive_pwl_breakpoints(
            R, q_lo, q_hi, epsilon=epsilon_pipe)
        lin_valve_q[v] = bq
        lin_valve_h[v] = bh
        valve_n_pts[v] = n_seg + 1

    lin_tank_q = {}
    lin_tank_h = {}
    tank_n_pts = {}
    for tl in tank_links:
        q_lo = min(tank_min[tl], -200)
        q_hi = max(tank_max[tl], 240)
        R = tankpipe_hw[tl]
        bq, bh, n_seg = adaptive_pwl_breakpoints(
            R, q_lo, q_hi, epsilon=epsilon_pipe)
        lin_tank_q[tl] = bq
        lin_tank_h[tl] = bh
        tank_n_pts[tl] = n_seg + 1
    lin_pump_q = {}
    lin_pump_h = {}
    pump_n_pts = {}
    for pump in pump_link:
        co = pump_co[pump]
        def func(q, _co=co): return pump_characteristic(_co, q)
        q_lo, q_hi = pump_q_ranges.get(pump, (pump_min[pump], pump_max[pump]))
        bq, bh, n_seg = adaptive_pwl_breakpoints_pump(
            func, q_lo, q_hi, epsilon=epsilon_pump)
        lin_pump_q[pump] = bq
        lin_pump_h[pump] = bh
        pump_n_pts[pump] = n_seg + 1
    try:
        model = gp.Model()
        optimization_log = []
        _current_incumbent = None
        _current_bd = -1e10

        def my_callback(model, where):
            nonlocal _current_incumbent, _current_bd
            if where == gp.GRB.Callback.MIPSOL:
                _current_incumbent = model.cbGet(gp.GRB.Callback.MIPSOL_OBJ)
                _current_bd = model.cbGet(gp.GRB.Callback.MIPSOL_OBJBND)
                work_time = model.cbGet(gp.GRB.Callback.RUNTIME)
                gap = abs(_current_incumbent - _current_bd) / \
                    (abs(_current_incumbent) + 1e-10)
                optimization_log.append({
                    'Event': 'MIPSOL',
                    'WorkingTime(s)': round(work_time, 4),
                    'Incumbent': round(_current_incumbent, 6),
                    'BestBD': round(_current_bd, 6),
                    'Gap(%)': round(gap * 100, 4)
                })
            elif where == gp.GRB.Callback.MIPNODE:
                if model.cbGet(
                        gp.GRB.Callback.MIPNODE_STATUS) == gp.GRB.OPTIMAL:
                    new_bd = model.cbGet(gp.GRB.Callback.MIPNODE_OBJBND)
                if new_bd > _current_bd + 1e-6:
                    _current_bd = new_bd
                    work_time = model.cbGet(gp.GRB.Callback.RUNTIME)
                    inc = round(_current_incumbent,
                                6) if _current_incumbent is not None else None
                    gap = abs(_current_incumbent - _current_bd) / (abs(_current_incumbent) + 1e-10) \
                        if _current_incumbent is not None else None
                    optimization_log.append({
                        'Event': 'MIPNODE',
                        'WorkingTime(s)': round(work_time, 4),
                        'Incumbent': inc,
                        'BestBD': round(_current_bd, 6),
                        'Gap(%)': round(gap * 100, 4) if gap is not None else None
                    })
        q_pipe = model.addVars(
            steps,
            pipe_link,
            lb=-500,
            vtype=gp.GRB.CONTINUOUS,
            name='q_pipe')
        h_pipe = model.addVars(
            steps,
            pipe_link,
            lb=-100,
            vtype=gp.GRB.CONTINUOUS,
            name='h_pipe')
        q_valve = model.addVars(
            steps,
            valve_link,
            lb=0,
            vtype=gp.GRB.CONTINUOUS,
            name='q_valve')
        h_valve = model.addVars(
            steps,
            valve_link,
            vtype=gp.GRB.CONTINUOUS,
            name='h_valve')
        s_valve = model.addVars(
            steps,
            valve_link,
            vtype=gp.GRB.BINARY,
            name='s_valve')
        p_jun = model.addVars(
            steps,
            jun_node,
            lb=-7,
            vtype=gp.GRB.CONTINUOUS,
            name='p_jun')
        h_tank = model.addVars(
            steps,
            tank_links,
            lb=-100,
            vtype=gp.GRB.CONTINUOUS,
            name='h_tank')
        q_tank = model.addVars(
            steps,
            tank_links,
            lb=-500,
            vtype=gp.GRB.CONTINUOUS,
            name='q_tank')
        p_tank = model.addVars(
            steps,
            tank_node,
            vtype=gp.GRB.CONTINUOUS,
            name='p_tank')
        x_tank_low = model.addVars(
            steps,
            tank_node,
            vtype=gp.GRB.BINARY,
            name='x_tank_low')
        x_tank_high = model.addVars(
            steps,
            tank_node,
            vtype=gp.GRB.BINARY,
            name='x_tank_high')
        s_tank = model.addVars(
            steps,
            tank_links,
            vtype=gp.GRB.BINARY,
            name='s_tank')
        q_pump = model.addVars(
            steps,
            pump_link,
            lb=0,
            vtype=gp.GRB.CONTINUOUS,
            name='q_pump')
        h_pump = model.addVars(
            steps,
            pump_link,
            vtype=gp.GRB.CONTINUOUS,
            name='h_pump')
        s_pump = model.addVars(
            steps,
            pump_link,
            vtype=gp.GRB.BINARY,
            name='s_pump')
        m_pump = model.addVars(
            steps,
            pump_link,
            vtype=gp.GRB.BINARY,
            name='m_pump')
        z_pump = model.addVars(
            steps,
            pump_link,
            vtype=gp.GRB.CONTINUOUS,
            name='z_pump')
        lamda_pump = {}
        x_pump = {}
        for pump in pump_link:
            n_pts_p = pump_n_pts[pump]
            n_seg_p = n_pts_p - 1
            x_dim_p = math.ceil(math.log2(max(n_seg_p, 1)))
            for t in range(steps):
                for seg in range(n_pts_p):
                    lamda_pump[t, pump, seg] = model.addVar(
                        lb=0, ub=1, vtype=gp.GRB.CONTINUOUS,
                        name=f'lamda_pump_{t}_{pump}_{seg}')
                for k in range(x_dim_p):
                    x_pump[t, pump, k] = model.addVar(
                        vtype=gp.GRB.BINARY, name=f'x_pump_{t}_{pump}_{k}')
        model.update()
        eff = 0.85
        tariff_pattern = ['pumptariff']
        ta_pattern = {}
        for pattern in tariff_pattern:
            [errcode, pattern_id] = enGetpatternindex(pattern)
            [errcode, length] = enGetpatternlen(pattern_id)
            if length != 24:
                print('Error: Tariff pattern length is not equal to simulation steps')
            ta_pattern[pattern] = []
            for i in range(length):
                [errcode, value] = enGetpatternvalue(pattern_id, i + 1)
                value_rounded = round(value, 3)
                ta_pattern[pattern].append(value_rounded)
        obj = 0
        for t in range(steps):
            for m in pump_link:
                pump_name = pump_id[m]
                pump = wn.get_link(pump_name)
                pattern_name = pump.energy_pattern
                if pattern_name in ta_pattern:
                    pattern = ta_pattern[pattern_name]
                else:
                    pattern = np.ones(steps)
                n_pts_m = pump_n_pts[m]
                for seg in range(n_pts_m):
                    obj += lamda_pump[t, m, seg] * (
                        9800 * pattern[t] * lin_pump_q[m][seg] * lin_pump_h[m][seg]) / (eff * (10**6))
        model.setObjective(obj, gp.GRB.MINIMIZE)
        for t in range(steps):
            for m in pump_link:
                n_pts_m = pump_n_pts[m]
                n_seg_m = n_pts_m - 1
                lam_m = [lamda_pump[t, m, seg] for seg in range(n_pts_m)]
                model.addConstr(q_pump[t, m] <= s_pump[t, m] * 300)
                model.addConstr(gp.quicksum(lam_m) == 1)
                n_bits_m = math.ceil(math.log2(max(n_seg_m, 1)))
                for bit in range(n_bits_m):
                    x_b = x_pump[t, m, bit]
                    lam_bit1, lam_bit0 = [], []
                    for seg in range(n_pts_m):
                        adj_segs = []
                        if seg > 0:
                            adj_segs.append(seg - 1)
                        if seg < n_seg_m:
                            adj_segs.append(seg)
                        bits_of_adj = [(k >> bit) & 1 for k in adj_segs]
                        if all(b == 1 for b in bits_of_adj):
                            lam_bit1.append(lam_m[seg])
                        elif all(b == 0 for b in bits_of_adj):
                            lam_bit0.append(lam_m[seg])
                    if lam_bit1:
                        model.addConstr(gp.quicksum(lam_bit1) <= x_b)
                    if lam_bit0:
                        model.addConstr(gp.quicksum(lam_bit0) <= 1 - x_b)
                model.addConstr(q_pump[t, m] == gp.quicksum(
                    lam_m[seg] * lin_pump_q[m][seg] for seg in range(n_pts_m)))
                model.addConstr(h_pump[t, m] == gp.quicksum(
                    lam_m[seg] * lin_pump_h[m][seg] for seg in range(n_pts_m)))
        for t in range(steps):
            for j in jun_node:
                Q_in = 0
                Q_out = 0
                for p in pipe_link:
                    if pipe_start[p] == j:
                        Q_out += q_pipe[t, p]
                if pipe_end[p] == j:
                    Q_in += q_pipe[t, p]
                for m in pump_link:
                    if pump_start[m] == j:
                        Q_out += q_pump[t, m]
                if pump_end[m] == j:
                    Q_in += q_pump[t, m]
                for v in valve_link:
                    if valve_start[v] == j:
                        Q_out += q_valve[t, v]
                if valve_end[v] == j:
                    Q_in += q_valve[t, v]
                for tank in tank_links:
                    if tank_start[tank] == j:
                        Q_out += q_tank[t, tank]
                if tank_end[tank] == j:
                    Q_in += q_tank[t, tank]
                model.addConstr(
                    Q_in - Q_out == demand[j][t],
                    name=f"demand_jun_{j}_{t}"
                )
        for t in range(steps):
            for p in pipe_link:
                if pipe_start[p] in res_node:
                    model.addConstr(q_pipe[t, p] >= 0)
            for tank in tank_node:
                model.addConstr(p_tank[t, tank] >= tank_minl[tank])
                model.addConstr(p_tank[t, tank] <= tank_maxl[tank])
        for tank in tank_node:
            for t in range(1, steps):
                q_in = 0
                q_out = 0
                for pipe in tank_links:
                    if tank_start[pipe] == tank:
                        q_out += q_tank[t - 1, pipe]
                if tank_end[pipe] == tank:
                    q_in += q_tank[t - 1, pipe]
                model.addConstr(
                    p_tank[t, tank] == p_tank[t - 1, tank] +
                    3.6 * (q_in - q_out) / tank_s[tank]
                )
        for tank in tank_node:
            q_in = 0
            q_out = 0
            t_end = steps - 1
            for pipe in tank_links:
                if tank_start[pipe] == tank:
                    q_out += q_tank[t_end, pipe]
                if tank_end[pipe] == tank:
                    q_in += q_tank[t_end, pipe]
            model.addConstr(
                p_tank[0, tank] <= p_tank[t_end, tank] +
                3.6 * (q_in - q_out) / tank_s[tank]
            )
        for t in range(1, steps):
            for tank in tank_node:
                model.addConstr(
                    tank_maxl[tank] - 300 * (1 - x_tank_high[t, tank]) <= p_tank[t - 1, tank])
                model.addConstr(p_tank[t - 1, tank] <= 300 *
                                x_tank_high[t, tank] + tank_maxl[tank])
                model.addConstr(
                    tank_minl[tank] - 300 * x_tank_low[t, tank] <= p_tank[t - 1, tank])
                model.addConstr(
                    p_tank[t - 1, tank] <= tank_minl[tank] + 300 * (1 - x_tank_low[t, tank]))
                model.addConstr(
                    x_tank_high[t, tank] + x_tank_low[t, tank] <= 1)
        for tank in tank_node:
            init_level = tank_init[tank]
            if init_level >= tank_maxl[tank]:
                model.addConstr(x_tank_high[0, tank] == 1)
                model.addConstr(x_tank_low[0, tank] == 0)
            elif init_level <= tank_minl[tank]:
                model.addConstr(x_tank_high[0, tank] == 0)
                model.addConstr(x_tank_low[0, tank] == 1)
            else:
                model.addConstr(x_tank_high[0, tank] == 0)
                model.addConstr(x_tank_low[0, tank] == 0)
        Q_max = 300
        for t in range(1, steps):
            for pipe in tank_links:
                i = tank_start[pipe]
                j = tank_end[pipe]
                if j in tank_node:
                    model.addConstr(-Q_max * A_bwd[pipe] <= q_tank[t, pipe])
                    model.addConstr(q_tank[t, pipe] <= Q_max * A_fwd[pipe])
                    model.addConstr(q_tank[t, pipe] <=
                                    Q_max * (1 - x_tank_high[t, j]))
                    model.addConstr(q_tank[t, pipe] >= -
                                    Q_max * (1 - x_tank_low[t, j]))
                elif i in tank_node:
                    model.addConstr(-Q_max * A_bwd[pipe] <= q_tank[t, pipe])
                    model.addConstr(q_tank[t, pipe] <= Q_max * A_fwd[pipe])
                    model.addConstr(q_tank[t, pipe] >= -
                                    Q_max * (1 - x_tank_high[t, i]))
                    model.addConstr(q_tank[t, pipe] <=
                                    Q_max * (1 - x_tank_low[t, i]))
        pwl_func = add_pwl_sos2_constraints
        lamda_pipe, x_pipe = pwl_func(
            model, steps, pipe_link, q_pipe, h_pipe,
            lin_pipe_q, lin_pipe_h, pipe_n_pts, 'pipe')
        lamda_valve, x_valve = pwl_func(
            model, steps, valve_link, q_valve, h_valve,
            lin_valve_q, lin_valve_h, valve_n_pts, 'valve')
        for t in range(steps):
            for v in valve_link:
                model.addConstr(q_valve[t, v] <= s_valve[t, v] * bigM_valve[v])
        lamda_tank, x_tank = pwl_func(
            model, steps, tank_links, q_tank, h_tank,
            lin_tank_q, lin_tank_h, tank_n_pts, 'tank')
        for t in range(steps):
            for p in pipe_link:
                if pipe_start[p] in jun_node:
                    model.addConstr(p_jun[t, pipe_start[p]] +
                                    jun_ele[pipe_start[p]] -
                                    (p_jun[t, pipe_end[p]] +
                                     jun_ele[pipe_end[p]]) -
                                    h_pipe[t, p] == 0)
                if pipe_start[p] in res_node:
                    model.addConstr(res_head[pipe_start[p]] -
                                    (p_jun[t, pipe_end[p]] +
                                     jun_ele[pipe_end[p]]) -
                                    h_pipe[t, p] == 0)
        for t in range(steps):
            for p in tank_links:
                if tank_start[p] in jun_node:
                    model.addConstr(p_jun[t, tank_start[p]] +
                                    jun_ele[tank_start[p]] -
                                    (p_tank[t, tank_end[p]] +
                                     tank_ele[tank_end[p]]) -
                                    h_tank[t, p] == 0)
                else:
                    model.addConstr(p_tank[t, tank_start[p]] +
                                    tank_ele[tank_start[p]] -
                                    (p_jun[t, tank_end[p]] +
                                     jun_ele[tank_end[p]]) -
                                    h_tank[t, p] == 0)
        for t in range(steps):
            for m in pump_link:
                M_m = bigM_pump[m]
                model.addConstr((p_jun[t, pump_start[m]] +
                                 jun_ele[pump_start[m]]) -
                                (p_jun[t, pump_end[m]] +
                                 jun_ele[pump_end[m]]) +
                                h_pump[t, m] <= M_m *
                                (1 -
                                 s_pump[t, m]))
                model.addConstr((p_jun[t, pump_start[m]] +
                                 jun_ele[pump_start[m]]) -
                                (p_jun[t, pump_end[m]] +
                                 jun_ele[pump_end[m]]) +
                                h_pump[t, m] >= -
                                M_m *
                                (1 -
                                 s_pump[t, m]))
        if max_switch is not None:
            for t in range(1, steps):
                for m in pump_link:
                    model.addConstr(
                        m_pump[t, m] + s_pump[t - 1, m] >= s_pump[t, m])
            for m in pump_link:
                model.addConstr(gp.quicksum(
                    m_pump[t, m] for t in range(steps)) <= max_switch)
        for t in range(steps):
            for v in valve_link:
                M_v = bigM_valve[v]
                model.addConstr((p_jun[t, valve_start[v]] +
                                 jun_ele[valve_start[v]] -
                                 (p_jun[t, valve_end[v]] +
                                  jun_ele[valve_end[v]]) -
                                 h_valve[t, v]) <= M_v *
                                (1 -
                                 s_valve[t, v]))
                model.addConstr((p_jun[t, valve_start[v]] +
                                 jun_ele[valve_start[v]] -
                                 (p_jun[t, valve_end[v]] +
                                  jun_ele[valve_end[v]]) -
                                 h_valve[t, v]) >= -
                                M_v *
                                (1 -
                                 s_valve[t, v]))
        for t in range(steps):
            for n in pressurenode:
                model.addConstr(p_jun[t, n] >= p_min)
                model.addConstr(p_jun[t, n] <= p_max)
        label = max_switch if max_switch is not None else 'no_limit'
        model.setParam(gp.GRB.Param.LogFile, f'mini_{label}.log')
        model.params.MIPFocus = 1
        model.params.FeasibilityTol = 1e-6
        model.setParam('MIPGap', 1e-4)
        model.params.IntFeasTol = 1e-5
        model.Params.timeLimit = 28800
        model.update()
        model.optimize(my_callback)
        if model.status == gp.GRB.INFEASIBLE:
            model.computeIIS()
            model.write("infeasible.ilp")
            for c in model.getConstrs():
                if c.IISConstr:
                    print(f"{c.ConstrName}")
        model.write(f'gb2.0_switch{max_switch}.sol')
        model.write(f'gb2.0_switch{max_switch}.lp')
        print("Obj:", model.objVal)

    except gp.GurobiError as e:
        print('Error code ' + str(e.errno) + ": " + str(e))

run_gurobi("mini.inp", 5)
