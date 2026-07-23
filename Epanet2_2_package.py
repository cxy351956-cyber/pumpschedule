from ctypes import *
import os
coding_C='GBK'#['utf-8','GBK']#垮平台编程，python为utf-8,C为GBK，python转C需使用GBK
current_path=os.path.abspath('.')
dll_path=current_path+'\\Epanet2_2_sim'
dll = cdll.LoadLibrary(dll_path)

#Project Functions
def enClose():
    errcode=dll.ENclose()
    return errcode

def enGetcount(countcode):
    count= pointer(c_int(0))
    errcode=dll.ENgetcount( c_int(countcode), count)
    count=count.contents.value
    return errcode,count

def enGettitle(out_line1, out_line2, out_line3):
    out_line1=(c_char * 1000)(*bytes(out_line1,coding_C))
    out_line2=(c_char * 1000)(*bytes(out_line2,coding_C))
    out_line3=(c_char * 1000)(*bytes(out_line3,coding_C))
    errcode=dll.ENgettitle (out_line1,out_line2, out_line3)
    return errcode

def enInit(rptFile,outFile,unitsType,headLossType):
    rptFile=(c_char * 1000)(*bytes(rptFile,coding_C))
    outFile=(c_char * 1000)(*bytes(outFile,coding_C))
    unitsType=c_int(unitsType)
    headLossType=c_int(headLossType)
    errcode=dll.ENInit(rptFile,outFile, unitsType,headLossType)
    return errcode

def enOpen(Inpfile,rtpfile,f3):
    inp=(c_char * 1000)(*bytes(Inpfile,coding_C))
    rtp=(c_char * 1000)(*bytes(rtpfile,coding_C))
    f_3=(c_char * 1000)(*bytes(f3,coding_C))
    errcode=dll.ENopen(inp,rtp, f_3)
    return errcode

def enSaveinpfile(Inpfile):
    inp=(c_char * 1000)(*bytes(Inpfile,coding_C))
    errcode=dll.ENsaveinpfile(inp)
    return errcode

def enSettitle(line1, line2, line3):
    line1=(c_char * 1000)(*bytes(line1,coding_C))
    line2=(c_char * 1000)(*bytes(line2,coding_C))
    line3=(c_char * 1000)(*bytes(line3,coding_C))
    errcode=dll.ENsettitle(line1,line2,line3)
    return errcode

#Hydraulic Analysis Functions
def enCloseH():
    errcode=dll.ENcloseH()
    return errcode

def enInitH(int_var=0):
    errcode=dll.ENinitH(c_int(int_var))
    return errcode

def enNextH(tstep):
    t=pointer(c_long(tstep))
    errcode=dll.ENnextH(t )
    t=t.contents.value
    return errcode,t

def enOpenH():
    errcode=dll.ENopenH()
    return errcode

def enRunH(time=0):
    T = pointer(c_long(time))
    errcode=dll.ENrunH(T)
    time = T.contents.value
    return [errcode,time]

def enSaveH():
    errcode=dll.ENsaveH()
    return errcode

def enSavehydfile(filename):
    filename=(c_char * 1000)(*bytes(filename,coding_C))
    errcode=dll.ENsavehydfile(filename)
    return errcode


def enCheckunlinked():
    errcode=dll.ENcheckunlinked()
    return errcode


def enSolveH():
    errcode=dll.ENcheckunlinked()
    if errcode>100:
        print('inp 文件存在错误，无法进行水力求解')
        return errcode

    errcode=dll.ENsolveH()
    totalerrcode=0
    if errcode==305:
        T = pointer(c_long(0))
        errcode=dll.ENopenH()
        totalerrcode=max(totalerrcode,errcode)

        errcode=dll.ENinitH(0)
        totalerrcode=max(totalerrcode,errcode)

        errcode=dll.ENrunH(T)
        totalerrcode=max(totalerrcode,errcode)

        errcode=dll.ENcloseH()
        totalerrcode=max(totalerrcode,errcode)
        if totalerrcode>100:
            print('Error in hydraulic simulation')
    return errcode



def enUsehydfile(filename):
    filename=(c_char * 1000)(*bytes(filename,coding_C))
    errcode=dll.ENusehydfile(filename)
    return errcode

#Water Quality Analysis Functions
def enCloseQ():
    errcode=dll.ENcloseQ()
    return errcode

def enInitQ(int_saveflag):
    errcode=dll.ENinitQ(c_int(int_saveflag))
    return errcode

def enNextQ(tstep):
    t=pointer(c_long(tstep))
    errcode=dll.ENnextQ(t)
    t=t.contents.value
    return errcode,t

def enOpenQ():
    errcode=dll.ENopenQ()
    return errcode

def enRunQ(time=0):
    t = pointer(c_long(time))
    errcode=dll.ENrunQ(t)
    t=t.contents.value
    return errcode,t

def enSolveQ():
    errcode=dll.ENsolveQ()
    return errcode

def enStepQ(tleft):
    t=pointer(c_long(tleft))
    errcode=dll.ENstepQ(t)
    t=t.contents.value
    return errcode,t

#Reporting Functions

#Analysis Options Functions
def enGetflowunits():
    unitscode=pointer(c_int(0))
    errcode=dll.ENgetflowunits(unitscode)
    unitscode=unitscode.contents.value
    return errcode,unitscode

def enGetoption(optioncode):
    value=pointer(c_float(0))
    errcode=dll.ENgetoption( c_int(optioncode), value)
    value=value.contents.value
    return errcode,value

def enGetqualinfo():
    qualType=pointer(c_int(0))
    chemName=(c_char * 1000)(*bytes("",coding_C))
    chemUnits=(c_char * 1000)(*bytes("",coding_C))
    traceNode=pointer(c_int(0))
    errcode=dll.ENgetqualinfo(qualType,chemName,chemUnits,traceNode)
   
    qualType=qualType.contents.value
    chemName=str(chemName.value, encoding = "utf-8")
    chemUnits=str(chemUnits.value, encoding = "utf-8")
    traceNode=traceNode.contents.value
    return errcode,qualType,chemName,chemUnits,traceNode

def enGetqualtype():
    qualType=pointer(c_int(0))
    traceNode=pointer(c_int(0))
    errcode=dll.ENgetqualtype(qualType, traceNode)
    qualType=qualType.contents.value
    traceNode=traceNode.contents.value
    return errcode,qualType,traceNode

def enGettimeparam(paramcode):
    timevalue=pointer(c_long(0))
    errcode=dll.ENgettimeparam(paramcode,timevalue)
    timevalue=timevalue.contents.value
    return errcode,timevalue

def enSetflowunits(units):
    units=c_int(units)
    errcode=dll.ENsetflowunits(units)
    units=units.contents.value
    return errcode,units

def enSetoption(optioncode,value):
    errcode=dll.ENsetoption(c_int(optioncode), c_float(value ))
    return errcode

def enSetqualtype(qualcode,chemname,chemunits,tracenode):
    chemname=(c_char * 1000)(*bytes(chemname,coding_C))
    chemunits=(c_char * 1000)(*bytes(chemunits,coding_C))
    tracenode=(c_char * 1000)(*bytes(tracenode,coding_C))
    errcode=dll.ENsetqualtype( c_int(qualcode),chemname,chemunits,tracenode )
    return errcode

def enSettimeparam(paramcode, timevalue):
    errcode=dll.ENsettimeparam( c_int(paramcode), c_long(timevalue)  )
    return errcode

#Network Node Functions
def enAddnode(id,nodeType):
    id=(c_char * 1000)(*bytes(id,coding_C))
    index=pointer(c_int(0))
    errcode=dll.ENaddnode(id, c_int(nodeType),index)
    return errcode,index.contents.value

def enDeletenode(index,actionCode):
    errcode=dll.ENdeletenode(c_int(index), c_int(actionCode))
    return errcode

def enGetcoord(index):
    x=pointer(c_double(0))#这里是double型的数据
    y=pointer(c_double(0))#这里是double型的数据
    errcode=dll.ENgetcoord(c_int(index),x,y)
    return errcode, x.contents.value,y.contents.value

def enGetnodeID(index):
    id=(c_char * 1000)(*bytes("",coding_C))#创建bytes 类型
    errcode=dll.ENgetnodeid(c_int(index),id)
    id=str(id.value, encoding = "utf-8")#bytes转string
    return errcode,id

def enGetnodeIndex(ID):
    id=(c_char * 1000)(*bytes(ID,coding_C))
    index = pointer(c_int(0))
    errcode=dll.ENgetnodeindex(id, index )
    index = index.contents.value
    return errcode,index

def enGetnodeType(index):
    nodeType= pointer(c_int(0))
    errcode=dll.ENgetnodetype(c_int(index), nodeType)
    return errcode,nodeType.contents.value

def enGetnodelink(node_index, i):
    linkindex= pointer(c_int(0))
    errcode=dll.ENgetnodelink(c_int(node_index),c_int(i),linkindex)
    linkindex = linkindex.contents.value
    return  errcode,linkindex

def enGetnodevalue(index,paramcode):


    value = pointer(c_float(0))
    errcode=dll.ENgetnodevalue(c_int(index),c_int(paramcode),value )
    value=value.contents.value
    return errcode,value

def enGetnodevalue_new(index,paramcode):
#注意：该函数通过对原函数封装，增加了读取水厂压力功能
    # 0  水厂标高
    # 28 水厂的总水头
    # 30 水厂压力
    if paramcode==30:#读取水厂压力时，需要两次调用
        errcode,elev=enGetnodevalue(index,0)
        errcode,head=enGetnodevalue(index,28)
        value=head-elev
    else:#其它情况直接调度原来函数
        errcode,value=enGetnodevalue(index,paramcode)

    return errcode,value

def enGettankdata(index):
    elev= pointer(c_float(0))#高程
    initlvl= pointer(c_float(0))#初始水位
    minlvl= pointer(c_float(0))#最小水位
    maxlvl= pointer(c_float(0))#最大水位
    diam= pointer(c_float(0))#水塔直径
    minvol= pointer(c_float(0))#最小容积
    errcode=dll.ENgettankdata(c_int(index), elev, initlvl, minlvl, maxlvl, diam, minvol)
    elev=elev.contents.value
    initlvl=initlvl.contents.value
    minlvl=minlvl.contents.value
    maxlvl=maxlvl.contents.value
    diam=diam.contents.value
    minvol=minvol.contents.value

    return errcode,elev,initlvl,minlvl,maxlvl,diam,minvol

def enSetcoord(index, x,  y):
    errcode=dll.ENsetcoord(c_int(index), c_double(x), c_double(y))#这里是double型的数据
    return errcode

def enSetjuncdata(index,elev,basedemand,demandPatternID):

    demandPatternID=(c_char * 1000)(*bytes(demandPatternID,coding_C))
    errcode=dll.ENsetjuncdata(c_int(index), c_float(elev), c_float(basedemand),demandPatternID)
    return errcode

def enSetnodeid(index,newid):
    
    newid=(c_char * 1000)(*bytes(newid,coding_C))
    errcode=dll.ENsetnodeid(c_int(index), newid)
    return errcode

def enSetnodevalue(index=0,paramcode=0,value=0):
    errcode=dll.ENsetnodevalue( c_int(index), c_int(paramcode), c_float(value))
    return errcode

def enSetnodevalue_new(index=0,paramcode=0,value=0):
    #注意：该函数通过对原函数封装，增加了水厂水头、压力、高程函数
    # 0  水厂标高
    # 28 水厂的总水头
    # 30 水厂压力
    if paramcode==30:#设置水厂压力时，需要两次调用
        errcode,elev=enGetnodevalue(index,0)#注意，必须先设置高程再设置压力
        head=elev+value
        errcode=enSetnodevalue(index,28,head)

    else:
        errcode=enSetnodevalue(index,paramcode,value)

    return errcode

def enSettankdata(index=0,elev=0,initlvl=0,minlvl=0,maxlvl=0,diam=0, minvol=0,volcurveID=''):
    volcurveID=(c_char * 1000)(*bytes(volcurveID,coding_C))
    errcode=dll.ENsettankdata(c_int(index), c_float(elev),c_float(initlvl), c_float(minlvl),c_float(maxlvl), c_float(diam), c_float(minvol), volcurveID)
    return errcode

#Nodal Demand Functions
def enAdddemand(nodeIndex,baseDemand,demandPatternID,demandName):
    demandPatternID=(c_char * 1000)(*bytes(demandPatternID,coding_C))
    demandName=(c_char * 1000)(*bytes(demandName,coding_C))
    errcode=dll.ENadddemand(c_int(nodeIndex), c_float(baseDemand),demandPatternID, demandName)
    return errcode


def enDeletedemand(nodeIndex,demandIndex):
    errcode=dll.ENdeletedemand(c_int(nodeIndex),c_int(demandIndex))
    return errcode

def enGetbasedemand(nodeIndex,demandIndex):
    baseDemand = pointer(c_float(0))
    errcode=dll.ENgetbasedemand(c_int(nodeIndex), c_int(demandIndex),baseDemand)
    return errcode,baseDemand.contents.value

def enGetdemandindex(nodeIndex,demandName):
    demandName=(c_char * 1000)(*bytes(demandName,coding_C))
    demandIndex=pointer(c_int(0))
    errcode=dll.ENgetdemandindex(c_int(nodeIndex), demandName, demandIndex)
    return errcode,demandIndex.contents.value

def enGetdemandmodel():
    model=pointer(c_int(0))
    pmin=pointer(c_float(0))
    preq=pointer(c_float(0))
    pexp=pointer(c_float(0))
    errcode=dll.ENgetdemandmodel(model,pmin,preq,pexp)
    model=model.contents.value
    pmin=pmin.contents.value
    preq=preq.contents.value
    pexp=pexp.contents.value
    return errcode,model,pmin,preq,pexp

def enGetdemandname(nodeIndex,demandIndex):
    demandName=(c_char * 1000)(*bytes("",coding_C))
    errcode=dll.ENgetdemandname(c_int(nodeIndex),c_int(demandIndex), demandName)
    demandName=str(demandName.value, encoding = "utf-8")
    return errcode,demandName


def enGetdemandpattern(nodeIndex,demandIndex):
    pattIdx=pointer(c_int(0))
    errcode=dll.ENgetdemandpattern(c_int(nodeIndex), c_int(demandIndex), pattIdx)
    return errcode,pattIdx.contents.value

def enGetnumdemands(nodeIndex):
    numDemands=pointer(c_int(0))
    errcode=dll.ENgetnumdemands(c_int(nodeIndex), numDemands)
    return errcode,numDemands.contents.value

def enSetbasedemand(nodeIndex,demandIndex,baseDemand):
    errcode=dll.ENsetbasedemand(c_int(nodeIndex), c_int(demandIndex),c_float(baseDemand))
    return errcode

def enSetdemandmodel(modeltype,pmin,preq,pexp):
    errcode=dll.ENsetdemandmodel(c_int(modeltype), c_float(pmin),c_float(preq),c_float(pexp))
    return errcode

def enSetdemandname(nodeIndex,demandIndex,demandName):
    demandName=(c_char * 1000)(*bytes(demandName,coding_C))
    errcode=dll.ENsetdemandname(c_int(nodeIndex), c_int(demandIndex), demandName)
    return errcode

def enSetdemandpattern(nodeIndex,demandIndex,patIndex):
    errcode=dll.ENsetdemandpattern(c_int(nodeIndex),c_int(demandIndex), c_int( patIndex))
    return errcode

#Network Link Functions
def enAddlink(nodeid, linkType, fromNodeid, toNodeid):
    nodeid=(c_char * 1000)(*bytes(nodeid,coding_C))
    fromNodeid=(c_char * 1000)(*bytes(fromNodeid,coding_C))
    toNodeid=(c_char * 1000)(*bytes(toNodeid,coding_C))
    index=pointer(c_int(0))
    errcode=dll.ENaddlink(nodeid, c_int(linkType), fromNodeid, toNodeid,index)
    return errcode,index.contents.value

def enDeletelink(index, actionCode):
    errcode=dll.ENdeletelink(c_int(index), c_int(actionCode))
    return errcode

def enGetheadcurveindex(pumpIndex):
    curveIndex=pointer(c_int(0))
    errcode=dll.ENgetheadcurveindex(c_int(pumpIndex), curveIndex)
    return errcode,curveIndex.contents.value

def enGetlinkID(index):
    id=(c_char * 1000)(*bytes("",coding_C))#创建bytes 类型
    errcode=dll.ENgetlinkid(c_int(index),id)
    id=str(id.value, encoding = "utf-8")#bytes转string
    return errcode,id

def enGetlinkIndex(ID):
    id=(c_char * 1000)(*bytes(ID,coding_C))
    index = pointer(c_int(0))
    errcode=dll.ENgetlinkindex(id, index )
    index = index.contents.value
    return errcode,index

def enGetlinkNodes(linkindex):
    fromnode= pointer(c_int(0))
    tonode = pointer(c_int(0))
    errcode=dll.ENgetlinknodes(linkindex, fromnode, tonode )
    fromnode=fromnode.contents.value
    tonode=tonode.contents.value
    return errcode,fromnode,tonode

def enGetlinkType(index):
    typecode = pointer(c_int(0))
    errcode=dll.ENgetlinktype( c_int(index), typecode)
    typecode= typecode.contents.value
    return errcode,typecode

def enGetlinkvalue(index,paramcode):
    value = pointer(c_float(0))
    errcode=dll.ENgetlinkvalue( c_int(index),c_int(paramcode),value )
    value=value.contents.value
    return errcode,value

def enGetpumptype(linkIndex):
    pumpType=pointer(c_int(0))
    errcode=dll.ENgetpumptype(c_int(linkIndex),pumpType)
    return errcode,pumpType.contents.value

def enSetheadcurveindex(linkIndex,curveIndex):
    errcode=dll.ENsetheadcurveindex(c_int(linkIndex), c_int(curveIndex))
    return errcode

def enSetlinkid(index,newid):
    newid=(c_char * 1000)(*bytes(newid,coding_C))
    errcode=dll.ENsetlinkid(c_int(index),newid)
    return errcode

def enSetlinknodes(linkIndex,startnodeIndex,endnodeIndex):
#Sets the indexes of a link's start- and end-nodes
    startnodeIndex=int(startnodeIndex)
    endnodeIndex=int(endnodeIndex)
    errcode=dll.ENsetlinknodes(c_int(linkIndex), c_int(startnodeIndex), c_int(endnodeIndex))
    return errcode

def enSetlinktype(in_index, linkType, actionCode):
#inout_index为一个列表，有两个元素分别为修改前后的索引
    in_index = pointer(c_int(in_index))#输入被修改管道的索引
    errcode=dll.ENsetlinktype(in_index,c_int(linkType),c_int(actionCode))
    out_index = in_index.contents.value#输出修改后管道索引
    return errcode,out_index

def enSetlinkvalue(index,paramcode,value):
    errcode=dll.ENsetlinkvalue(c_int(index), c_int(paramcode), c_float(value))
    return errcode

def enSetpipedata(index,length,diam,rough,mloss):
    errcode=dll.ENsetpipedata(c_int(index), c_float(length),c_float(diam), c_float(rough), c_float(mloss))
    return errcode

#Time Pattern Functions
def enAddpattern(patternID):
    patternID=(c_char * 1000)(*bytes(patternID,coding_C))
    errcode=dll.ENaddpattern(patternID)
    return errcode

def enDeletepattern(index):
    errcode=dll.ENdeletepattern(c_int(index))
    return errcode

def enGetaveragepatternvalue(index):
    value=pointer(c_float(0))
    errcode=dll.ENgetaveragepatternvalue(c_int(index), value)
    return errcode,value.contents.value

def enGetpatternid(patternIndex):
    id=(c_char * 1000)(*bytes("",coding_C))#创建bytes 类型
    errcode=dll.ENgetpatternid(c_int(patternIndex), id)
    id=str(id.value, encoding = "utf-8")#bytes转string
    return errcode,id
  
def enGetpatternindex(patternID):
    patternID=(c_char * 1000)(*bytes(patternID,coding_C))
    index=pointer(c_int(0))
    errcode=dll.ENgetpatternindex(patternID, index)
    return errcode,index.contents.value

def enGetpatternlen(index):
    len= pointer(c_int(0))
    errcode=dll.ENgetpatternlen(index, len)
    len=len.contents.value
    return errcode,len

def enGetpatternvalue(index,period):
    value = pointer(c_float(0))
    errcode=dll.ENgetpatternvalue(c_int(index),c_int(period),value )
    value =value.contents.value
    return errcode,value

def enSetpattern(index,factors,nfactors):
    factors=(c_float * nfactors)(*factors)#把列表传入变长,稍后与上面列表传入方式对比，确定哪一个是最合适的！
    errcode=dll.ENsetpattern(c_int(index), factors, c_int(nfactors ))
    return errcode

def enSetpatternid(index, patternID):
    patternID=(c_char * 1000)(*bytes(patternID,coding_C))
    errcode=dll.ENsetpatternid(c_int(index), patternID)
    return errcode

def enSetpatternvalue(index, period, value):
    errcode=dll.ENsetpatternvalue(c_int(index), c_int(period), c_float(value))
    return errcode

#Data Curve Functions
def enAddcurve(id):
    id=(c_char * 1000)(*bytes(id,coding_C))
    errcode=dll.ENaddcurve(id)
    return errcode

def enDeletecurve(index):
    errcode=dll.ENdeletecurve(c_int(index))
    return errcode

def enGetcurveID(index):
    id=(c_char * 1000)(*bytes("",coding_C))#创建bytes 类型
    errcode=dll.ENgetcurveid(c_int(index),id)
    id=str(id.value, encoding = "utf-8")#bytes转string
    return errcode,id

def enGetcurveindex(curveId):
    curveId=(c_char * 1000)(*bytes(curveId,coding_C))
    index=pointer(c_int(0))
    errcode=dll.ENgetcurveindex(curveId, index)
    return errcode, index.contents.value

def enGetcurvelen(index):
    len=pointer(c_int(0))
    errcode=dll.ENgetcurvelen(c_int(index),len)
    return errcode,len.contents.value

def enGetcurvetype(index):
    type=pointer(c_int(0))
    errcode=dll.ENgetcurvetype(c_int(index),type)
    return errcode,type.contents.value

def enGetcurvevalue(curveIndex,pointIndex):
    x=pointer(c_float(0))
    y=pointer(c_float(0))
    errcode=dll.ENgetcurvevalue(c_int(curveIndex),c_int(pointIndex), x,y)
    return errcode, x.contents.value,y.contents.value

def enSetcurvevalue(curveIndex,pointIndex,x,y):
    errcode=dll.ENsetcurvevalue(c_int(curveIndex),c_int(pointIndex), c_float(x),c_float(y))
    return errcode

def enSetcurveid(index,id):
    id=(c_char * 1000)(*bytes(id,coding_C))
    errcode=dll.ENsetcurveid(c_int(index),id)
    return errcode

def enSetcurvevalue(curveIndex,pointIndex,x,y):
    errcode=dll.ENsetcurvevalue(c_int(curveIndex),c_int(pointIndex), c_float(x),c_float(y))
    return errcode

def enAddControl(controlType,linkIndex,setting,nodeIndex,level):
    index = pointer(c_int(0))
    errcode=dll.ENaddcontrol(c_int(controlType),c_int(linkIndex),c_float(setting),c_int(nodeIndex),c_float(level),index)
    return errcode,index.contents.value

def enDeleteControl(controlIndex):
    errcode=dll.ENdeletecontrol(c_int(controlIndex))
    return errcode

def enGetControl(controlIndex):
    controlIndex=c_int(-10000)
    type = pointer(c_int(-10000))
    linikindex = pointer(c_int(-10000))
    setting = pointer(c_float(-10000))
    nodeindex = pointer(c_int(-10000))
    level = pointer(c_float(-10000))
    errcode=dll.ENgetcontrol(controlIndex,type,linikindex,setting,nodeindex,level)

    type=type.contents.value
    linikindex=linikindex.contents.value
    setting =setting.contents.value
    nodeindex =nodeindex.contents.value
    level =level.contents.value
    return errcode,type,linikindex,setting,nodeindex,level

def enSetControl(controlIndex,type,linkIndex,setting,nodeIndex,level):
    errcode=dll.ENsetcontrol(c_int(enDeleteControl),c_int(type),c_int(linkIndex),c_float(setting),c_int(nodeIndex),c_float(level))
    return errcode

