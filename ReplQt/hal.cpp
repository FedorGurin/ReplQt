#include "hal.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <stdlib.h>

#include "layerSpecRTM.h"
#include "layerMIL.h"
#include "layerRk.h"
#include "layerAnalog.h"
#include "chdev.h"
#include "vec3D.h"
#include <ctime>
#ifdef VXWORKS_PLATFORM
#include <net/utils/ifconfig.h>
#include <Ar16t16rApi.h>
#include <MonNik_api.h>
#include <sockLib.h>
#include "tftpLib.h"
#include "ioLib.h"
#include "rebootLib.h"
#include "usrFsLib.h"


#include <ipProto.h>
#include <net/if.h>
#include "inetLib.h"
#include "ioLib.h"
#include <sys/ioctl.h>
#endif


#include "glHAL.hpp"
#include "arServ.h"
#include "servUpdate.h"
#include "threadManagerVx.h"

HAL* HAL::hal = 0;
HAL* HAL::obj() {
    if (hal == 0)
        hal = new HAL();

    return hal;
}
float freqHz(int fr)
{
    fr+=1000;
    return fr;
}

#ifdef VXWORKS_PLATFORM
#define PATH_TO_CONF_ISA        "/ata0a/conf/confISA.bin"
#define PATH_TO_CH_TABLE        "/ata0a/conf/chTable.bin"
#define PATH_TO_PARAM_TABLE     "/ata0a/conf/paramTable.bin"
#define PATH_TO_ETH_TABLE       "/ata0a/conf/ethTable.bin"
#define PATH_TO_NAME_TABLE      "/ata0a/conf/nameTable.bin"
#define PATH_TO_ETH_PORT_TABLE  "/ata0a/conf/ethPortTable.bin"
#define PATH_TO_PACK_TABLE      "/ata0a/conf/packTable.bin"

#define PATH_TO_VER_FILE    "/ata0a/ver.txt"
#else
#define PATH_TO_CONF_ISA        "./ata0a/conf/confISA.bin"
#define PATH_TO_CH_TABLE        "./ata0a/conf/chTable.bin"
#define PATH_TO_PARAM_TABLE     "./ata0a/conf/paramTable.bin"
#define PATH_TO_ETH_TABLE       "./ata0a/conf/ethTable.bin"
#define PATH_TO_NAME_TABLE      "./ata0a/conf/nameTable.bin"
#define PATH_TO_ETH_PORT_TABLE  "./ata0a/conf/ethPortTable.bin"
#define PATH_TO_PACK_TABLE      "./ata0a/conf/packTable.bin"
#endif
//const uint8_t maxOu = 30;
//const uint8_t maxSub = 32;
//const uint8_t maxWord = 32;
HAL::HAL() {
    eisaAvailable = false;
    milAvailable = false;
    eisa = new VxConnectToEISA;
    //! выделение памяти
    //!
    //! обнуление данных
    memset((char*) &confISA,     0, sizeof(confISA));
    memset((char*) &chTable,     0, sizeof(chTable));
    memset((char*) &paramTable,  0, sizeof(paramTable));
    memset((char*) &ethTable,    0, sizeof(ethTable));
    memset((char*) &nameTable,   0, sizeof(nameTable));
    memset((char*) &setup,       0, sizeof(setup));
    memset((char*) &statusPacket,0, sizeof(statusPacket));
    memset((char*) &packTable,   0, sizeof(packTable));
    memset((char*) &status,      0, sizeof(status));
    
    //! тригеры на отключение МКИО по каналам
    trigUpdateOS  = new Trigger<uint8_t>(0);
}
void HAL::initQ()
{
    //! создаем поток
    std::string pthrName = "pthServUpdate";// + std::(ar->freqArray);
    ThreadSeparate* thServUpdate = new ThreadSeparate(pthrName,
                                                          new ServUpdate(ID_SERV_UPDATE,1.0/12.5));
    thServUpdate->create();
}
//! создание директорий для файлов модели
void HAL::createDir()
{
    if(ls("/ata0a/conf/",FALSE) == ERROR)
    {
        int status = mkdir("/ata0a/conf");
        if(status == ERROR)
            std::cout << "HAL: Can't create dir [ata0a/conf]" << std::endl;
        else
            std::cout << "HAL: Create dir [ata0a/conf]" << std::endl;
    }
}
int HAL::uploadFile(std::string localPath, std::string nameFile)
{
    std::string fullNameFile = localPath + nameFile;
    int fd = open ((char*)fullNameFile.c_str(), 0x201, 0644);    
    if(fd != ERROR)
    {
       int status = tftpCopy ("host", 0, (char*)nameFile.c_str(), "get", "binary", fd);
       if(status == ERROR)
         std::cout<<std::setw(30)<<std::left<<"HAL: can`t upload - "<<nameFile<<std::endl;
       close(fd);       
    }
    return fd;
}
void HAL::init() {
    std::cout << std::endl;
    //! создание директорий для файлов модели
    createDir();
    //! зачитываем таблицы из файлов
    readConfFiles();
    //! зачитываем тестовые варианты таблиц
    loadTestTables();
    //! идентификация объекта
    identifySelf();
    //! поднимаем сетевые  интерфейсы
    upNetIf();
    //! инициализация подсистемы статусов
    initStatusSystem();
    //! создание потока для обеспечения обслуживания
    initQ();
    //! инициализация плат PCI
    initPCI();
    //! инициализация EISA, если подключена
    initEISA();
    //! инициализация плат ISA
    initISA();
    //! создаем каналы для обмена данных
    createChAR();
    //! создаем объекты для обмена по МКИО
    createChMIL();
    std::cout << "========================================" << std::endl;
}
void HAL::checkConnections()
{
    statusPacket.typeNode = HAL::obj()->typeCurrentNode;
    statusPacket.idNode   = HAL::obj()->currentNode;
    statusPacket.status   = 0;
    //! кол-во сетевых интерфейсов с другими узлами
    uint8_t numIf = HAL::obj()->numEthIf(HAL::obj()->typeCurrentNode,
                                         HAL::obj()->currentNode);
    //! цикл по доступным типам узлов
    for(int i = 0; i< numIf; i++)
    {
        //! выделяем сетевой интерфейс с каким ПВ связан
        TEthIf *eth = HAL::obj()->findEthIf(HAL::obj()->typeCurrentNode,
                                            HAL::obj()->currentNode,i);

        if(eth == 0)
            continue;
        //! определяем ip адрес этого узла
        TEthIf *eth1= HAL::obj()->findEthIf(eth->toTypeNode,
                                            eth->toIdNode,
                                            HAL::obj()->typeCurrentNode,
                                            HAL::obj()->currentNode);

        if(eth1 == 0)
            continue;
        //! проверяю соединение для данных интерфейсов(пинг или не пинг!?)
        int bytes = udpSend->sendTo((uint8_t*)&statusPacket,
                        sizeof(statusPacket),
                        HAL::obj()->ethPort(ETH_PORT_REC_CHECK_HAL), eth1->ip_int);
        if(bytes == ERROR)
            eth1->isConnect = 0;
        else
            eth1->isConnect = 1;
    }
    //! проверить пришли ли ответы
    recCheckConnection();
}

void HAL::recCheckConnection()
{
    int bytes = ERROR;

    do
    {
        bytes = ERROR;
        memset((void*)&statusPacket,0,sizeof(statusPacket));
        //! считываем заголовок
        bytes = udpRec->reciveFrom((uint8_t*)&statusPacket, sizeof(statusPacket), MSG_WAITALL );

        //! определяем ip адрес этого узла
        TEthIf *eth= HAL::obj()->findEthIf(HAL::obj()->typeCurrentNode,
                                           HAL::obj()->currentNode,
                                           statusPacket.typeNode,
                                           statusPacket.idNode);
        if(eth !=0 && statusPacket.status == 0)
            eth->isConnect = 2;
    }while(bytes>0);
}
void HAL::initStatusSystem()
{
    //! получение данных от других узлов
    udpRec = new VxTemplateUdpSocket;
    udpRec->init(HAL::obj()->ethPort(ETH_PORT_REC_CHECK_HAL),"");
    udpRec->bindTo();
    udpRec->setBlock(false);

    //! сокет для отправки в другие узлы
    udpSend = new VxTemplateUdpSocket();
    udpSend->init(HAL::obj()->ethPort(ETH_PORT_REC_CHECK_HAL),"");
}
void HAL::calculate()
{
//    //! триггер на признак "обновления версии"
//    trigUpdateOS->setState(setup.updateVer);

//    if(trigUpdateOS->isHighFront() == true)
//    {
//        updateVerOS();
//    }
    //checkConnections();
}
void HAL::initEISA() {
    std::cout << "========================================" << std::endl;
    std::cout << "HAL: Scaning EISA ..." << std::endl;
    eisa->startSearch();
    eisaAvailable = eisa->checkAvailable();
    if (eisaAvailable == true)
        std::cout << "EISA is connected" << std::endl;
    else
        std::cout << "EISA not found" << std::endl;
}
void HAL::initISA() {
    std::cout << "========================================" << std::endl;
    std::cout << "HAL: Scaning ISA adapters..." << std::endl;

    //! инициализация библиотеки работы с РК
    LayerRK::obj();
    //! инициализация библиотеки работы с ЦАП
    //LayerDAC::obj();
    //! инициализация библиотеки работы с АЦП
    //LayerADC::obj();
    //! иммитация аналоговых сигналов(ИМ сопотивлений, потенциометра и т.д)
    LayerAnalog::obj();
}
extern bool InitMonitorMil(void);
extern void InitRegimeMonitor(uint8_t indexAdapter);
void HAL::initPCI() {

#ifndef VXWORKS_SIM
    std::cout << "========================================" << std::endl;
    std::cout << "HAL: Scaning PCI adapters ..." << std::endl;
    std::cout << std::setw(30) << std::left << "HAL: Detected AR16T16R .... ";
    if (LayerConfAr::initApi() == false)
        std::cout << " Channels" << " [NOT FOUND]" << std::endl;
    else
        std::cout << LayerConfAr::maxNumCh() << " Channels" << " [READY]" << std::endl;
    //! инициализация адаптеров МКИО
    std::cout << std::setw(30) << std::left << "HAL: Detected MOM4 .... ";
    milAvailable = LayerConfMIL::initApi();
    if (milAvailable == false)
        std::cout << " 0 Channels" << " [NOT FOUND]" << std::endl;
    else
        std::cout << LayerConfMIL::maxNumCh() << " Channels" << " [READY]" << std::endl;

    //! инициализация 4 канальных мониторов
    for(int i = 0; i<nameTable.numNode; i++)
    {
        for(int j = 0;j<confPCI.numMil; j++)
        {
            if(nameTable.node[i].typeNode == confPCI.mil[j].typeNode &&
               nameTable.node[i].idNode == confPCI.mil[j].idNode)
            {
                for(int k = 0; k < confPCI.mil[j].numAdapterRegMon; k++)
                {
                    //! каналы МКИО
                    int index = confPCI.mil[j].numAdapterRegOU + k;
                    InitRegimeMonitor(index);
                    std::cout << std::setw(30) << std::left << "HAL: adapter "
                              << index <<" MOM4 set Monitor regime " <<std::endl;
                }
            }
        }
    }
#endif
}
bool HAL::checkConfFile(std::string nameFile)
{
    int fdRkIn = open(nameFile.c_str(), O_RDONLY, 0644);
    if (fdRkIn == ERROR) 
        return false;
    return true;
}
bool HAL::readConfFromFile(std::string nameFile, uintptr_t *data, uint32_t len) {

    //#define VXWORKS_PLATFORM
#ifdef VXWORKS_PLATFORM

    //! чтение РК
    int fdRkIn = open(nameFile.c_str(), O_RDONLY, 0644);
    if (fdRkIn == ERROR) {
        std::cout << std::setw(20)
                << "HAL: readConfFromFile(): Can`t open file -"
                << nameFile.c_str() << std::endl;
        return false;
    }
    lseek(fdRkIn, 0, SEEK_SET);
    int bytes = read(fdRkIn, (char*) data, len);
    if (bytes < 0) {
        std::cout << std::setw(20)
                << "HAL: readConfFromFile(): Can`t read file -"
                << nameFile.c_str() << std::endl;

        return false;
    }
#endif
    return true;
}
void HAL::upNetIf() {
    for (int i = 0; i < ethTable.numEth; i++) {
        if (ethTable.eth[i].fromIdNode == currentNode
                && ethTable.eth[i].fromTypeNode == typeCurrentNode
                && ethTable.eth[i].numIf > 0) {
            int status = ipAttach(ethTable.eth[i].numIf, "gei");
            if (status == ERROR)
                std::cout << "HAL: upNetIf() Can`t create eth interface"
                        << std::endl;
            std::string nameIf, nameIp;
            nameIf.append(ethTable.eth[i].nameIf);
            nameIp.append(ethTable.eth[i].ip);
            std::string arg = nameIf + " inet add " + nameIp
                    + " netmask 255.255.255.0";
            status = ifconfig((char*) arg.c_str());
            if (status == ERROR)
                std::cout << "HAL: ifconfig() Fail setting network interface"
                        << std::endl;
            arg = nameIf + " up";
            status = ifconfig((char*) arg.c_str());
            if (status == ERROR)
                std::cout << "HAL: Faild up interface" << std::endl;
        }
        ethTable.eth[i].ip_int = inet_addr(ethTable.eth[i].ip);
    }
}
void HAL::readConfFiles() {
     
    std::cout << "========================================" << std::endl;
    std::cout << "HAL: Loading Configuration data" << std::endl;
    if(checkConfFile(PATH_TO_CONF_ISA) == false)
        uploadFile("/ata0a/conf/","confISA.bin");
    readConfFromFile(PATH_TO_CONF_ISA, (uintptr_t *) &confISA, sizeof(confISA));
        
    if(checkConfFile(PATH_TO_CH_TABLE) == false)
        uploadFile("/ata0a/conf/","chTable.bin");
    readConfFromFile(PATH_TO_CH_TABLE, (uintptr_t *) &chTable, sizeof(chTable));
    
    if(checkConfFile(PATH_TO_PARAM_TABLE) == false)
        uploadFile("/ata0a/conf/","paramTable.bin");    
    readConfFromFile(PATH_TO_PARAM_TABLE, (uintptr_t *) &paramTable,       sizeof(paramTable));
       
    if(checkConfFile(PATH_TO_ETH_TABLE) == false)
        uploadFile("/ata0a/conf/","ethTable.bin");
    readConfFromFile(PATH_TO_ETH_TABLE, (uintptr_t *) &ethTable,  sizeof(ethTable));    
       
    if(checkConfFile(PATH_TO_NAME_TABLE) == false)
        uploadFile("/ata0a/conf/","nameTable.bin");
    readConfFromFile(PATH_TO_NAME_TABLE, (uintptr_t *) &nameTable, sizeof(nameTable));
    
    if(checkConfFile(PATH_TO_NAME_TABLE) == false)
        uploadFile("/ata0a/","ver.txt");
    readConfFromFile(PATH_TO_VER_FILE, (uintptr_t *) status.verModel, sizeof(status.verModel));
    
    if(checkConfFile(PATH_TO_ETH_PORT_TABLE) == false)
        uploadFile("/ata0a/conf/","ethPortTable.bin");
    readConfFromFile(PATH_TO_ETH_PORT_TABLE, (uintptr_t *) &ethPortTable, sizeof(ethPortTable));        
    
    if(checkConfFile(PATH_TO_PACK_TABLE) == false)
        uploadFile("/ata0a/conf/","packTable.bin");
    readConfFromFile(PATH_TO_PACK_TABLE, (uintptr_t *) &packTable,       sizeof(packTable));}

void HAL::identifySelf() {
    struct ifreq ifr;
    int fd;
    std::string curMac;
#ifdef CV1_SIM
    curMac = CV_1;
#endif
#ifdef CV3_SIM
    curMac = CV_3;
#endif
#ifndef VXWORKS_SIM
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == ERROR) {
        std::cout << "HAL::identifySelf(): Didn`t open socket" << std::endl;
        return;
    }
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ - 1);
    int flag = ioctl(fd, SIOCGIFLLADDR, &ifr);
    if (flag == ERROR) {
        std::cout << "HAL::identifySelf(): Can`t get MAC addres for eth0"
                << std::endl;
        return;
    }
    unsigned char* mac = (unsigned char*) ifr.ifr_addr.sa_data;

    char mas[2];
    for (int i = 0; i < 6; i++) {
        sprintf(mas, "%02X", mac[i]);
        curMac += mas;
        if (i < 5)
            curMac += "-";
    }
#endif
    for (int i = 0; i < nameTable.numNode; i++) {
        if (strcmp(nameTable.node[i].mac, curMac.c_str()) == 0) {
            typeCurrentNode = nameTable.node[i].typeNode;
            currentNode     = nameTable.node[i].idNode;
            currentNameNode = nameTable.node[i].name;
            if (typeCurrentNode == E_NODE_CV && currentNode == 1)
                currentGlobalTypeNode = ID_NODE_CV_1;
            else if (typeCurrentNode == E_NODE_CV && currentNode == 2)
                currentGlobalTypeNode = ID_NODE_CV_2;
            else if (typeCurrentNode == E_NODE_PV && currentNode == 1)
                currentGlobalTypeNode = ID_NODE_PV_1;
            else if (typeCurrentNode == E_NODE_PV && currentNode == 2)
                currentGlobalTypeNode = ID_NODE_PV_2;
            else if (typeCurrentNode == E_NODE_PV && currentNode == 3)
                currentGlobalTypeNode = ID_NODE_PV_3;
            else if (typeCurrentNode == E_NODE_PV && currentNode == 4)
                currentGlobalTypeNode = ID_NODE_PV_4;
            else if (typeCurrentNode == E_NODE_CV && currentNode == 3)
                currentGlobalTypeNode = ID_NODE_CV_3;
            else if (typeCurrentNode == E_NODE_PV && currentNode == 5)
                currentGlobalTypeNode = ID_NODE_PV_5;
            else if (typeCurrentNode == E_NODE_PV && currentNode == 9)
                currentGlobalTypeNode = ID_NODE_PV_RP;
        }
    }
    std::cout << "========================================" << std::endl;
    std::cout << "HAL: Loading Identify information ..." << std::endl;
    std::cout << "HAL: Node ID    = " << currentNameNode.c_str() << std::endl;
    std::cout << "HAL: Ver. Model = " << status.verModel << std::endl;

    close(fd);
}
void HAL::createChMIL()
{
    TDesMIL *mil = 0;
    TTableChHAL* table = &(HAL::obj()->chTable);
    for (int i = 0; i < table->numCh; i++)
    {
        TCh *ch = &(HAL::obj()->chTable.ch[i]);
        if (ch->typeNode == HAL::obj()->typeCurrentNode && ch->idNode == HAL::obj()->currentNode)
        {
            if (ch->typeCh == E_CH_MIL)
            {
                mil = (TDesMIL*) (ch->desData);
                LayerOuMIL::enable(ch->setting.numCh, mil->addr, ch->setting.numAdapter, 1);
                TDesMIL *mil = (TDesMIL*) ch->desData;
                LayerOuMIL::writeToAdapter(ch->setting.numAdapter,ch->setting.numCh, mil->addr, mil->subAddr, mil->s.ui32);
                LayerOuMIL::enable(ch->setting.numCh, mil->addr, ch->setting.numAdapter, 0);
                //LayerOuMIL::clearSubAddr(ch->setting.numCh, mil->addr, ch->setting.numAdapter, 1);
            }
        }
    }
}
void HAL::createChAR() {
    TDesAr *ar = 0;

    TTableChHAL* table = &(HAL::obj()->chTable);

    for (int i = 0; i < table->numCh; i++)
    {
        TCh *ch = &(HAL::obj()->chTable.ch[i]);
        if (ch->typeNode == typeCurrentNode && ch->idNode == currentNode)
        {
            if (ch->typeCh == E_CH_AR)
            {
                ar = (TDesAr*) (ch->desData);
                if(ar->freqArray > 0)
                {
                    //! создаем поток
                    ICalculateElement *arServ = new ArServ(ID_AR_SERV,i,1./ar->freqArray);
                    std::string pthrName = "pthArFreq";// + std::(ar->freqArray);
                    ThreadSeparate* thArHal = new ThreadSeparate(pthrName,arServ);
                    thArHal->create();
                    ThreadManagerVx::obj()->append(thArHal);
                }
                uint8_t bufAddr[MAX_ADR_AR];
                uint8_t numAddr=0;

                for(int j = 0;j<ar->numIndAddr[ar->curIndArray];j++)
                {
                    bufAddr[j] = ar->addr[ar->indAddr[j][ar->curIndArray]];
                    numAddr++;
                }
                //! инициализируем канал
                LayerArinc::initCh(ch->setting.numCh, bufAddr, numAddr, ch->setting.ioCh, ar->freq, ar->type, ar->rev);
                //if(ch->setting.ioCh == 0)
               // {
                    ioctlHAL(i,  IO_SWITCH_ON_OFF,1);
                    ch->setting.activeLocal = 1;
                    ar->active = 1;
               // }
            }
        }
    }
}
void HAL::setToAdapters()
{
    HAL* t = HAL::obj();
    TCh* ch = 0;
    //! записываю данные, если они принадлежат данному узлу
    for (int i = 0; i < t->chTable.numCh; i++)
    {
        ch = &(t->chTable.ch[i]);
        if (ch->idNode == currentNode && ch->typeNode == typeCurrentNode &&
            ch->setting.ioCh == 1 &&
            ch->setting.readyToWrite == 1 &&
            ch->setting.blockWrite == 0)
        {
            ch->setting.readyToWrite = 0;
            //! аппаратная запись на данном узле
            switch (ch->typeCh)
            {
            case E_CH_MIL:
            {
                TDesMIL *mil = (TDesMIL*) ch->desData;
                if(mil->active!= ch->setting.activeLocal)
                {
                    ioctlHAL(i,  IO_SWITCH_ON_OFF,mil->active);
                    ch->setting.activeLocal = mil->active;
                }
                
                LayerOuMIL::writeToAdapter(ch->setting.numAdapter,
                        ch->setting.numCh, mil->addr, mil->subAddr, mil->s.ui32);
                break;
            }
            case E_CH_AR: {
                TDesAr *ar = (TDesAr*) ch->desData;

                //! если пришла команда на включение
                if(ar->active!= ch->setting.activeLocal)
                {
                    ioctlHAL(i,  IO_SWITCH_ON_OFF,ar->active);
                    ch->setting.activeLocal = ar->active;
                }

                for(int i = 0;i<ar->numIndAddr[ar->curIndArray];i++)
                {
                    LayerArinc::writeTo(ch->setting.numCh, ch->setting.ioCh,
                            ar->value[ar->indAddr[i][ar->curIndArray]], ar->addr[ar->indAddr[i][ar->curIndArray]], i);
                }

//                for (int j = 0; j < ar->numAddr[ar->indexAddr]; j++)
//                {
//                    LayerArinc::writeTo(ch->setting.numCh, ch->setting.ioCh,
//                            ar->value[j], ar->addr[j][ar->indexAddr], j);
//                }
                break;
            }
            case E_CH_RK: {
                //LayerRK::set()
                break;
            }

            };

        } else if (eisaAvailable == true && ch->typeNode == E_NODE_EISA)
        {
            if (ch->typeCh == E_CH_RK) {
                //TAdapterISA * isa = findIsaAdapter(E_NODE_EISA, 1, E_A_TRREL48);
                //! создаем запрос по указаному каналу
                //eisa->sendRequest(i);
                //eisa->writeTo(isa->baseAddr[ch->setting.numAdapter], ,false);

            }
            //! отправляем данные в EISA
            //eisa->writeTo(0x120,(char*)&value0, 4,false);

        } else if (ch->typeNode == E_NODE_EISA) {
            //! определяем узел который имеет доступ к узлу EISA

        } else {
            //! отправляю данные к узлу, который владеет данными каналами
            //            if(ch->setting.ready == 1)
            //            {
            //             //   addToReq(ch);
            //                ch->setting.ready = 0;
            //            }
        }
    }
    //! запись сразу блоком
    LayerRK::obj()->writeToDevice();
    LayerAnalog::obj()->writeToDevice();
}
void HAL::getFromAdapters() {
    HAL* t = HAL::obj();
    TCh* ch = 0;
    //! записываю данные, если они принадлежат данному узлу
    for (int i = 0; i < t->chTable.numCh; i++)
    {
        ch = &(t->chTable.ch[i]);
        if (ch->idNode == currentNode &&
            ch->typeNode == typeCurrentNode  &&
            ch->setting.ioCh == 0 &&
            ch->setting.blockRead == 0) {
            //! аппаратная запись на данном узле
            switch (ch->typeCh) {
            case E_CH_MIL: {
                TDesMIL *mil = (TDesMIL*) ch->desData;
                if(mil->typeTrans == MON)
                    LayerMonMIL::readFromAdapter(ch->setting.numAdapter, ch->setting.numCh, mil->addr, mil->subAddr, mil->s.ui32);
                else
                    LayerOuMIL::readFromAdapter(ch->setting.numAdapter, ch->setting.numCh, mil->addr, mil->subAddr, mil->s.ui32);
                break;
            }
            case E_CH_AR: {
                TDesAr *ar = (TDesAr*) ch->desData;
                for(int i = 0;i<ar->numIndAddr[ar->curIndArray];i++)
                {

                    ar->value[ar->indAddr[i][ar->curIndArray]]= LayerArinc::readFrom(ch->setting.numCh, ch->setting.ioCh, ar->addr[ar->indAddr[i][ar->curIndArray]], ar->addr[ar->indAddr[i][ar->curIndArray]]);
                    //writeTo(ch->setting.numCh, ch->setting.ioCh,        , ar->addr[ar->indAddr[i][ar->curIndArray]], j);
                }

                break;
            }
            case E_CH_RK:
            {

                break;
            }

            };

        } else if (eisaAvailable == true && ch->typeNode == E_NODE_EISA) {
            //! отправляем данные в EISA

            //eisa->
            switch (ch->typeCh) {
            case E_CH_RK: {

                //                        uint16_t index = findIsaAdapter();
                //                        eisa->writeTo(ch->setting.numAdapter);
                break;
            }

            };

        } else if (ch->typeNode == E_NODE_EISA) {
            //! определяем узел который имеет доступ к узлу EISA

        } else {
            //! отправляю данные к узлу, который владеет данными каналами

        }
    }
    //! чтение сразу блоком
    LayerRK::obj()->readFromDevice();
    LayerAnalog::obj()->readFromDevice();
}
void HAL::loadTestTables() {
    ConfArTable arTable;
    HAL* t = HAL::obj();
    TDesMIL *mil     = 0;
    //TDesAr *ar       = 0;
    TDesIR *ir       = 0;
    TDesDAC *dac     = 0;
    TDesGen_U *gen_u = 0;
    TDesITP *itp     = 0;

    t->confPCI.numMil = 3;
    t->confPCI.mil[0].typeNode = E_NODE_CV;
    t->confPCI.mil[0].idNode   = 1;
    t->confPCI.mil[0].numAdapterRegOU  = 1;
    t->confPCI.mil[0].numAdapterRegMon = 0;
    t->confPCI.mil[0].numAdapterRegKOU = 0;

    t->confPCI.mil[1].typeNode = E_NODE_CV;
    t->confPCI.mil[1].idNode   = 3;
    t->confPCI.mil[1].numAdapterRegOU  = 1;
    t->confPCI.mil[1].numAdapterRegMon = 0;
    t->confPCI.mil[1].numAdapterRegKOU = 0;

    t->confPCI.mil[2].typeNode = E_NODE_PV;
    t->confPCI.mil[2].idNode   = 10;
    t->confPCI.mil[2].numAdapterRegOU  = 0;
    t->confPCI.mil[2].numAdapterRegMon = 1;
    t->confPCI.mil[2].numAdapterRegKOU = 0;

    //! кол-во адаптеров
    /*t->confISA.numAdapters = 20;
    //-----------------------------------------------------
    t->confISA.isa[0].typeNode = E_NODE_EISA;
    t->confISA.isa[0].idNode = 0;
    t->confISA.isa[0].typeAdapter = E_A_GEN;
    t->confISA.isa[0].numBaseAddr = 2;
    t->confISA.isa[0].baseAddr[0] = 0x100;
    t->confISA.isa[0].baseAddr[1] = 0x110;
    t->confISA.isa[0].codeCheck = 0x1100;
    t->confISA.isa[0].maskCheck = 0xFFFF;
    t->confISA.isa[0].maxChAdapter = 4;
    t->confISA.isa[0].offsetCheck = 4;
    //-----------------------------------------------------
    t->confISA.isa[1].typeNode = E_NODE_EISA;
    t->confISA.isa[1].idNode = 0;
    t->confISA.isa[1].typeAdapter = E_A_TRREL48;
    t->confISA.isa[1].numBaseAddr = 4;
    t->confISA.isa[1].baseAddr[0] = 0x120;
    t->confISA.isa[1].baseAddr[1] = 0x130;
    t->confISA.isa[1].baseAddr[2] = 0x140;
    t->confISA.isa[1].baseAddr[3] = 0x150;
    t->confISA.isa[1].codeCheck = 0x7070;
    t->confISA.isa[1].maskCheck = 0xFFFF;
    t->confISA.isa[1].maxChAdapter = 48;
    t->confISA.isa[1].offsetCheck = 6;

    //------------------------------------------------------
    t->confISA.isa[2].typeNode = E_NODE_EISA;
    t->confISA.isa[2].idNode = 0;
    t->confISA.isa[2].typeAdapter = E_A_IR;
    t->confISA.isa[2].numBaseAddr = 2;
    t->confISA.isa[2].baseAddr[0] = 0x160;
    t->confISA.isa[2].baseAddr[1] = 0x170;
    t->confISA.isa[2].codeCheck = 0xF000;
    t->confISA.isa[2].maskCheck = 0xFFF0;
    t->confISA.isa[2].maxChAdapter = 12;
    t->confISA.isa[2].offsetCheck = 4;
    //------------------------------------------------------
    t->confISA.isa[3].typeNode = E_NODE_EISA;
    t->confISA.isa[3].idNode = 0;
    t->confISA.isa[3].typeAdapter = E_A_RSREL48;
    t->confISA.isa[3].numBaseAddr = 2;
    t->confISA.isa[3].baseAddr[0] = 0x180;
    t->confISA.isa[3].baseAddr[1] = 0x190;
    t->confISA.isa[3].codeCheck = 0x0707;
    t->confISA.isa[3].maskCheck = 0xFFFF;
    t->confISA.isa[3].maxChAdapter = 48;
    t->confISA.isa[3].offsetCheck = 6;
    //------------------------------------------------------
    t->confISA.isa[4].typeNode = E_NODE_EISA;
    t->confISA.isa[4].idNode = 0;
    t->confISA.isa[4].typeAdapter = E_A_DAC16;
    t->confISA.isa[4].numBaseAddr = 3;
    t->confISA.isa[4].baseAddr[0] = 0x1b0;
    t->confISA.isa[4].baseAddr[1] = 0x1a0;
    t->confISA.isa[4].baseAddr[2] = 0x1c0;
    t->confISA.isa[4].codeCheck = 0x4000;
    t->confISA.isa[4].maskCheck = 0xE000;
    t->confISA.isa[4].maxChAdapter = 16;
    t->confISA.isa[4].offsetCheck = 0;
    //------------------------------------------------------
    t->confISA.isa[5].typeNode = E_NODE_EISA;
    t->confISA.isa[5].idNode = 0;
    t->confISA.isa[5].typeAdapter = E_A_IP;
    t->confISA.isa[5].numBaseAddr = 1;
    t->confISA.isa[5].baseAddr[0] = 0x1d0;
    t->confISA.isa[5].codeCheck = 0xF000;
    t->confISA.isa[5].maskCheck = 0xFFF0;
    t->confISA.isa[5].maxChAdapter = 16;
    t->confISA.isa[5].offsetCheck = 4;
    //------------------------------------------------------
    t->confISA.isa[6].typeNode = E_NODE_EISA;
    t->confISA.isa[6].idNode = 0;
    t->confISA.isa[6].typeAdapter = E_A_ITP;
    t->confISA.isa[6].numBaseAddr = 5;
    t->confISA.isa[6].baseAddr[0] = 0x1e0;
    t->confISA.isa[6].baseAddr[1] = 0x1f0;
    t->confISA.isa[6].baseAddr[2] = 0x200;
    t->confISA.isa[6].baseAddr[3] = 0x210;
    t->confISA.isa[6].baseAddr[4] = 0x220;
    t->confISA.isa[6].codeCheck = 0x114;
    t->confISA.isa[6].maxChAdapter = 2;
    t->confISA.isa[6].maskCheck = 0xFFFF;
    t->confISA.isa[6].offsetCheck = 0;
    ///////////////ПВ 3
    t->confISA.isa[7].typeNode = E_NODE_PV;
    t->confISA.isa[7].idNode = 3;
    t->confISA.isa[7].typeAdapter = E_A_TRREL48;
    t->confISA.isa[7].numBaseAddr = 1;
    t->confISA.isa[7].baseAddr[0] = 0x210;
    t->confISA.isa[7].codeCheck = 0x7070;
    t->confISA.isa[7].maskCheck = 0xFFFF;
    t->confISA.isa[7].offsetCheck = 6;

    t->confISA.isa[8].typeNode = E_NODE_PV;
    t->confISA.isa[8].idNode = 3;
    t->confISA.isa[8].typeAdapter = E_A_RSREL48;
    t->confISA.isa[8].numBaseAddr = 1;
    t->confISA.isa[8].baseAddr[0] = 0x220;
    t->confISA.isa[8].codeCheck = 0x7070;
    t->confISA.isa[8].maskCheck = 0xFFFF;
    t->confISA.isa[8].offsetCheck = 6;

    t->confISA.isa[9].typeNode = E_NODE_PV;
    t->confISA.isa[9].idNode = 3;
    t->confISA.isa[9].typeAdapter = E_A_ADC32;
    t->confISA.isa[9].numBaseAddr = 1;
    t->confISA.isa[9].baseAddr[0] = 0x230;*/
    /*HAL::obj()->confISA.isa[9].codeCheck    = 0x7070;
     HAL::obj()->confISA.isa[9].maskCheck    = 0xFFFF;
     HAL::obj()->confISA.isa[9].offsetCheck  = 6;*/

    /*t->confISA.isa[10].typeNode = E_NODE_PV;
    t->confISA.isa[10].idNode = 3;
    t->confISA.isa[10].typeAdapter = E_A_DAC16;
    t->confISA.isa[10].numBaseAddr = 1;
    t->confISA.isa[10].baseAddr[0] = 0x240;
    t->confISA.isa[10].codeCheck = 0x4000;
    t->confISA.isa[10].maskCheck = 0xE000;
    t->confISA.isa[10].maxChAdapter = 16;
    t->confISA.isa[10].offsetCheck = 0;

    t->confISA.isa[11].typeNode = E_NODE_PV;
    t->confISA.isa[11].idNode = 1;
    t->confISA.isa[11].typeAdapter = E_A_TRREL48;
    t->confISA.isa[11].numBaseAddr = 1;
    t->confISA.isa[11].baseAddr[0] = 0x210;
    t->confISA.isa[11].codeCheck = 0x7070;
    t->confISA.isa[11].maskCheck = 0xFFFF;
    t->confISA.isa[11].offsetCheck = 6;

    t->confISA.isa[12].typeNode = E_NODE_PV;
    t->confISA.isa[12].idNode = 1;
    t->confISA.isa[12].typeAdapter = E_A_RSREL48;
    t->confISA.isa[12].numBaseAddr = 1;
    t->confISA.isa[12].baseAddr[0] = 0x220;
    t->confISA.isa[12].codeCheck = 0x7070;
    t->confISA.isa[12].maskCheck = 0xFFFF;
    t->confISA.isa[12].offsetCheck = 6;

    t->confISA.isa[13].typeNode = E_NODE_PV;
    t->confISA.isa[13].idNode = 3;
    t->confISA.isa[13].typeAdapter = E_A_DACWAV;
    t->confISA.isa[13].numBaseAddr = 1;
    t->confISA.isa[13].baseAddr[0] = 0x300;
    t->confISA.isa[13].codeCheck = 0xDAC0;
    t->confISA.isa[13].maskCheck = 0xFF70;
    t->confISA.isa[13].offsetCheck = 4;
    //! для ЛЛ
    t->confISA.isa[14].typeNode = E_NODE_PV;
    t->confISA.isa[14].idNode = 5;
    t->confISA.isa[14].typeAdapter = E_A_TRREL48;
    t->confISA.isa[14].numBaseAddr = 1;
    t->confISA.isa[14].baseAddr[0] = 0x210;
    t->confISA.isa[14].codeCheck = 0x7070;
    t->confISA.isa[14].maskCheck = 0xFFFF;
    t->confISA.isa[14].offsetCheck = 6;

    t->confISA.isa[15].typeNode = E_NODE_PV;
    t->confISA.isa[15].idNode = 5;
    t->confISA.isa[15].typeAdapter = E_A_RSREL48;
    t->confISA.isa[15].numBaseAddr = 1;
    t->confISA.isa[15].baseAddr[0] = 0x220;
    t->confISA.isa[15].codeCheck = 0x7070;
    t->confISA.isa[15].maskCheck = 0xFFFF;
    t->confISA.isa[15].offsetCheck = 6;

    t->confISA.isa[16].typeNode = E_NODE_PV;
    t->confISA.isa[16].idNode = 5;
    t->confISA.isa[16].typeAdapter = E_A_ADC32;
    t->confISA.isa[16].numBaseAddr = 1;
    t->confISA.isa[16].baseAddr[0] = 0x230;*/
       /*HAL::obj()->confISA.isa[9].codeCheck    = 0x7070;
        HAL::obj()->confISA.isa[9].maskCheck    = 0xFFFF;
        HAL::obj()->confISA.isa[9].offsetCheck  = 6;*/

    /*t->confISA.isa[17].typeNode = E_NODE_PV;
    t->confISA.isa[17].idNode = 5;
    t->confISA.isa[17].typeAdapter = E_A_DAC16;
    t->confISA.isa[17].numBaseAddr = 1;
    t->confISA.isa[17].baseAddr[0] = 0x240;
    t->confISA.isa[17].codeCheck = 0x4000;
    t->confISA.isa[17].maskCheck = 0xE000;
    t->confISA.isa[17].maxChAdapter = 16;
    t->confISA.isa[17].offsetCheck = 0;

    t->confISA.isa[18].typeNode = E_NODE_PV;
    t->confISA.isa[18].idNode = 4;
    t->confISA.isa[18].typeAdapter = E_A_TRREL48;
    t->confISA.isa[18].numBaseAddr = 1;
    t->confISA.isa[18].baseAddr[0] = 0x210;
    t->confISA.isa[18].codeCheck = 0x7070;
    t->confISA.isa[18].maskCheck = 0xFFFF;
    t->confISA.isa[18].offsetCheck = 6;

    t->confISA.isa[19].typeNode = E_NODE_PV;
    t->confISA.isa[19].idNode = 4;
    t->confISA.isa[19].typeAdapter = E_A_RSREL48;
    t->confISA.isa[19].numBaseAddr = 1;
    t->confISA.isa[19].baseAddr[0] = 0x220;
    t->confISA.isa[19].codeCheck = 0x7070;
    t->confISA.isa[19].maskCheck = 0xFFFF;
    t->confISA.isa[19].offsetCheck = 6;



    //! список портов для обмена данными по ethernet
    t->ethPortTable.numPort = 18;
    t->ethPortTable.port[ETH_PORT_SEND_LMI_KGOO]    = 7030;
    t->ethPortTable.port[ETH_PORT_REC_LMI_KGOO]     = 9500;
    t->ethPortTable.port[ETH_PORT_REC_JOY]          = 9000;
    t->ethPortTable.port[ETH_PORT_REC_MPPM]         = 8881;
    t->ethPortTable.port[ETH_PORT_JNODE]            = 9999;
    t->ethPortTable.port[ETH_PORT_SEND_KGOO_ETH]    = 7030;
    t->ethPortTable.port[ETH_PORT_SEND_KGOO_PAL]    = 7070;
    t->ethPortTable.port[ETH_PORT_SEND_PKIP]        = 10001;
    t->ethPortTable.port[ETH_PORT_REC_PKIP]         = 10090;
    t->ethPortTable.port[ETH_PORT_SEND_ARM_IO]      = 65000;
    t->ethPortTable.port[ETH_PORT_REC_ARM_IO ]      = 0;
    t->ethPortTable.port[ETH_PORT_SEND_PKIP58]      = 5500;
    t->ethPortTable.port[ETH_PORT_REC_PKIP58]       = 5501;
    t->ethPortTable.port[ETH_PORT_REC_CHECK_HAL]    = 4131;
    t->ethPortTable.port[ETH_PORT_SEND_LMI_PW]      = 4131;
    t->ethPortTable.port[ETH_PORT_SEND_KGOO_ETH_2]  = 7030;
    t->ethPortTable.port[ETH_PORT_SEND_KGOO_RMP_1]  = 7030;
    t->ethPortTable.port[ETH_PORT_SEND_KGOO_RMP_2]  = 7030;


    
    //! таблица Ethernet каналов
    t->ethTable.numEth = 38;

    t->ethTable.eth[0].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[0].fromIdNode = 2;
    t->ethTable.eth[0].numIf = 1;
    t->ethTable.eth[0].toTypeNode = E_NODE_PV;
    t->ethTable.eth[0].toIdNode = 1;
    strcpy(t->ethTable.eth[0].ip, "192.168.91.201");
    t->ethTable.eth[0].ip_int = inet_addr(t->ethTable.eth[0].ip);
    strcpy(t->ethTable.eth[0].nameIf, "gei1");

    t->ethTable.eth[1].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[1].fromIdNode = 2;
    t->ethTable.eth[1].numIf = 2;
    t->ethTable.eth[1].toTypeNode = E_NODE_PV;
    t->ethTable.eth[1].toIdNode = 2;
    strcpy(t->ethTable.eth[1].ip, "192.168.92.202");
    t->ethTable.eth[1].ip_int = inet_addr(t->ethTable.eth[1].ip);
    strcpy(t->ethTable.eth[1].nameIf, "gei2");

    t->ethTable.eth[2].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[2].fromIdNode = 2;
    t->ethTable.eth[2].numIf = 3;
    t->ethTable.eth[2].toTypeNode = E_NODE_PV;
    t->ethTable.eth[2].toIdNode = 3;
    strcpy(t->ethTable.eth[2].ip, "192.168.93.203");
    t->ethTable.eth[2].ip_int = inet_addr(t->ethTable.eth[2].ip);
    strcpy(t->ethTable.eth[2].nameIf, "gei3");

    t->ethTable.eth[3].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[3].fromIdNode = 2;
    t->ethTable.eth[3].numIf = 4;
    t->ethTable.eth[3].toTypeNode = E_NODE_PV;
    t->ethTable.eth[3].toIdNode = 4;
    strcpy(t->ethTable.eth[3].ip, "192.168.94.204");
    t->ethTable.eth[3].ip_int = inet_addr(t->ethTable.eth[3].ip);
    strcpy(t->ethTable.eth[3].nameIf, "gei4");

    t->ethTable.eth[4].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[4].fromIdNode = 2;
    t->ethTable.eth[4].numIf = 5;
    t->ethTable.eth[4].toTypeNode = E_NODE_PV;
    t->ethTable.eth[4].toIdNode = 9;
    strcpy(t->ethTable.eth[4].ip, "192.168.95.205");
    t->ethTable.eth[4].ip_int = inet_addr(t->ethTable.eth[4].ip);
    strcpy(t->ethTable.eth[4].nameIf, "gei5");

    t->ethTable.eth[5].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[5].fromIdNode = 2;
    t->ethTable.eth[5].numIf = 6;
    t->ethTable.eth[5].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[5].toIdNode = 0;
    strcpy(t->ethTable.eth[5].ip, "192.168.80.200");
    t->ethTable.eth[5].ip_int = inet_addr(t->ethTable.eth[5].ip);
    strcpy(t->ethTable.eth[5].nameIf, "gei6");


    //! описание ПВ3
    t->ethTable.eth[6].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[6].fromIdNode = 3;
    t->ethTable.eth[6].numIf = 1;
    t->ethTable.eth[6].toTypeNode = E_NODE_CV;
    t->ethTable.eth[6].toIdNode = 1;
    strcpy(t->ethTable.eth[6].ip, "192.168.83.31");
    t->ethTable.eth[6].ip_int = inet_addr(t->ethTable.eth[6].ip);
    strcpy(t->ethTable.eth[6].nameIf, "gei1");

    t->ethTable.eth[7].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[7].fromIdNode = 3;
    t->ethTable.eth[7].numIf = 2;
    t->ethTable.eth[7].toTypeNode = E_NODE_CV;
    t->ethTable.eth[7].toIdNode = 2;

    strcpy(t->ethTable.eth[7].ip, "192.168.93.32");
    t->ethTable.eth[7].ip_int = inet_addr(t->ethTable.eth[7].ip);
    strcpy(t->ethTable.eth[7].nameIf, "gei2");

    //! Описание для Eth ЦВ1
    t->ethTable.eth[8].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[8].fromIdNode = 1;
    t->ethTable.eth[8].numIf = 1;
    t->ethTable.eth[8].toTypeNode = E_NODE_PV;
    t->ethTable.eth[8].toIdNode = 1;
    strcpy(t->ethTable.eth[8].ip, "192.168.81.101");
    t->ethTable.eth[8].ip_int = inet_addr(t->ethTable.eth[8].ip);
    strcpy(t->ethTable.eth[8].nameIf, "gei1");

    t->ethTable.eth[9].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[9].fromIdNode = 1;
    t->ethTable.eth[9].numIf = 2;
    t->ethTable.eth[9].toTypeNode = E_NODE_PV;
    t->ethTable.eth[9].toIdNode = 2;
    strcpy(t->ethTable.eth[9].ip, "192.168.82.102");
    t->ethTable.eth[9].ip_int = inet_addr(t->ethTable.eth[9].ip);
    strcpy(t->ethTable.eth[9].nameIf, "gei2");

    t->ethTable.eth[10].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[10].fromIdNode = 1;
    t->ethTable.eth[10].numIf = 3;
    t->ethTable.eth[10].toTypeNode = E_NODE_PV;
    t->ethTable.eth[10].toIdNode = 3;

    strcpy(t->ethTable.eth[10].ip, "192.168.83.103");
    t->ethTable.eth[10].ip_int = inet_addr(t->ethTable.eth[10].ip);
    strcpy(t->ethTable.eth[10].nameIf, "gei3");

    t->ethTable.eth[11].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[11].fromIdNode = 1;
    t->ethTable.eth[11].numIf = 4;
    t->ethTable.eth[11].toTypeNode = E_NODE_PV;
    t->ethTable.eth[11].toIdNode = 4;

    strcpy(t->ethTable.eth[11].ip, "192.168.84.104");
    t->ethTable.eth[11].ip_int = inet_addr(t->ethTable.eth[11].ip);
    strcpy(t->ethTable.eth[11].nameIf, "gei4");

    t->ethTable.eth[12].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[12].fromIdNode = 1;
    t->ethTable.eth[12].toTypeNode = E_NODE_PV;
    t->ethTable.eth[12].toIdNode = 9;
    t->ethTable.eth[12].numIf = 5;
    strcpy(t->ethTable.eth[12].ip, "192.168.85.105");
    t->ethTable.eth[12].ip_int = inet_addr(t->ethTable.eth[12].ip);
    strcpy(t->ethTable.eth[12].nameIf, "gei5");


    //! описание ПВ1
    t->ethTable.eth[13].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[13].fromIdNode = 1;
    t->ethTable.eth[13].numIf = 1;
    t->ethTable.eth[13].toTypeNode = E_NODE_CV;
    t->ethTable.eth[13].toIdNode = 1;
    strcpy(t->ethTable.eth[13].ip, "192.168.81.11");
    t->ethTable.eth[13].ip_int = inet_addr(t->ethTable.eth[13].ip);
    strcpy(t->ethTable.eth[13].nameIf, "gei1");

    t->ethTable.eth[14].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[14].fromIdNode = 1;
    t->ethTable.eth[14].numIf = 2;
    t->ethTable.eth[14].toTypeNode = E_NODE_CV;
    t->ethTable.eth[14].toIdNode = 2;
    strcpy(t->ethTable.eth[14].ip, "192.168.91.12");
    strcpy(t->ethTable.eth[14].nameIf, "gei2");

    //! для обмена с EISA
    t->ethTable.eth[15].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[15].fromIdNode = 1;
    t->ethTable.eth[15].numIf = 3;
    t->ethTable.eth[15].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[15].toIdNode = 2;
    strcpy(t->ethTable.eth[15].ip, "192.168.1.1");
    strcpy(t->ethTable.eth[15].nameIf, "gei3");

    //! описание ПВ2
    t->ethTable.eth[16].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[16].fromIdNode = 2;
    t->ethTable.eth[16].numIf = 1;
    t->ethTable.eth[16].toTypeNode = E_NODE_CV;
    t->ethTable.eth[16].toIdNode = 1;
    strcpy(t->ethTable.eth[16].ip, "192.168.82.21");
    strcpy(t->ethTable.eth[16].nameIf, "gei1");

    t->ethTable.eth[17].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[17].fromIdNode = 2;
    t->ethTable.eth[17].numIf = 2;
    t->ethTable.eth[17].toTypeNode = E_NODE_CV;
    t->ethTable.eth[17].toIdNode = 2;
    strcpy(t->ethTable.eth[17].ip, "192.168.92.22");
    strcpy(t->ethTable.eth[17].nameIf, "gei2");

    //! описание ПВ4
    t->ethTable.eth[18].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[18].fromIdNode = 4;
    t->ethTable.eth[18].numIf = 1;
    t->ethTable.eth[18].toTypeNode = E_NODE_CV;
    t->ethTable.eth[18].toIdNode = 1;
    strcpy(t->ethTable.eth[18].ip, "192.168.84.41");
    strcpy(t->ethTable.eth[18].nameIf, "gei1");

    t->ethTable.eth[19].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[19].fromIdNode = 4;
    t->ethTable.eth[19].numIf = 2;
    t->ethTable.eth[19].toTypeNode = E_NODE_CV;
    t->ethTable.eth[19].toIdNode = 2;
    strcpy(t->ethTable.eth[19].ip, "192.168.94.42");
    strcpy(t->ethTable.eth[19].nameIf, "gei2");

    t->ethTable.eth[ETH_IP_LMI].fromTypeNode = E_NODE_LMI;
    t->ethTable.eth[ETH_IP_LMI].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_LMI].numIf = 0;
    t->ethTable.eth[ETH_IP_LMI].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_LMI].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_LMI].ip, "192.168.72.155");
    strcpy(t->ethTable.eth[ETH_IP_LMI].nameIf, "gei0");

    t->ethTable.eth[21].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[21].fromIdNode = 2;
    t->ethTable.eth[21].numIf = 0;
    t->ethTable.eth[21].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[21].toIdNode = 0;
    strcpy(t->ethTable.eth[21].ip, "192.168.72.2");
    strcpy(t->ethTable.eth[21].nameIf, "gei0");

    t->ethTable.eth[22].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[22].fromIdNode = 1;
    t->ethTable.eth[22].numIf = 0;
    t->ethTable.eth[22].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[22].toIdNode = 0;
    strcpy(t->ethTable.eth[22].ip, "192.168.72.1");
    strcpy(t->ethTable.eth[22].nameIf, "gei0");

    t->ethTable.eth[23].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[23].fromIdNode = 1;
    t->ethTable.eth[23].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[23].toIdNode = 0;
    t->ethTable.eth[23].numIf = 6;
    strcpy(t->ethTable.eth[23].ip, "192.168.71.1");
    strcpy(t->ethTable.eth[23].nameIf, "gei6");

    t->ethTable.eth[ETH_IP_KGOO].fromTypeNode = E_NODE_KGOO;
    t->ethTable.eth[ETH_IP_KGOO].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_KGOO].numIf = 0;
    t->ethTable.eth[ETH_IP_KGOO].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_KGOO].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_KGOO].ip, "192.168.71.100");
    strcpy(t->ethTable.eth[ETH_IP_KGOO].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_PKIP].fromTypeNode = E_NODE_PKIP;
    t->ethTable.eth[ETH_IP_PKIP].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_PKIP].numIf = 0;
    t->ethTable.eth[ETH_IP_PKIP].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_PKIP].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_PKIP].ip, "192.168.71.200");
    strcpy(t->ethTable.eth[ETH_IP_PKIP].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_PKIP_58].fromTypeNode = E_NODE_PKIP;
    t->ethTable.eth[ETH_IP_PKIP_58].fromIdNode = 2;
    t->ethTable.eth[ETH_IP_PKIP_58].numIf = 0;
    t->ethTable.eth[ETH_IP_PKIP_58].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_PKIP_58].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_PKIP_58].ip, "192.168.71.201");
    strcpy(t->ethTable.eth[ETH_IP_PKIP_58].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_ARM_IO].fromTypeNode = E_NODE_PKIP;
    t->ethTable.eth[ETH_IP_ARM_IO].fromIdNode = 2;
    t->ethTable.eth[ETH_IP_ARM_IO].numIf = 0;
    t->ethTable.eth[ETH_IP_ARM_IO].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_ARM_IO].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_ARM_IO].ip, "192.168.71.255");
    strcpy(t->ethTable.eth[ETH_IP_ARM_IO].nameIf, "gei0");


    t->ethTable.eth[28].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[28].fromIdNode = 4;
    t->ethTable.eth[28].numIf = 3;
    t->ethTable.eth[28].toTypeNode = E_NODE_CV;
    t->ethTable.eth[28].toIdNode = 2;
    strcpy(t->ethTable.eth[28].ip, "192.168.71.40");
    strcpy(t->ethTable.eth[28].nameIf, "gei3");

    t->ethTable.eth[29].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[29].fromIdNode = 3;
    t->ethTable.eth[29].numIf = 0;
    t->ethTable.eth[29].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[29].toIdNode = 0;
    strcpy(t->ethTable.eth[29].ip, "192.168.72.3");
    strcpy(t->ethTable.eth[29].nameIf, "gei0");

    //! Описание для Eth ЦВ3(для ЛЛ)
    t->ethTable.eth[30].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[30].fromIdNode = 3;
    t->ethTable.eth[30].numIf = 1;
    t->ethTable.eth[30].toTypeNode = E_NODE_PV;
    t->ethTable.eth[30].toIdNode = 5;
    strcpy(t->ethTable.eth[30].ip, "192.168.85.3");
    strcpy(t->ethTable.eth[30].nameIf, "gei1");

    //! Описание для Eth ЦВ3(для ЛЛ)
    t->ethTable.eth[31].fromTypeNode = E_NODE_CV;
    t->ethTable.eth[31].fromIdNode = 3;
    t->ethTable.eth[31].numIf = 2;
    t->ethTable.eth[31].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[31].toIdNode = 0;
    strcpy(t->ethTable.eth[31].ip, "192.168.71.3");
    strcpy(t->ethTable.eth[31].nameIf, "gei2");

    t->ethTable.eth[32].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[32].fromIdNode = 5;
    t->ethTable.eth[32].numIf = 1;
    t->ethTable.eth[32].toTypeNode = E_NODE_CV;
    t->ethTable.eth[32].toIdNode = 3;
    strcpy(t->ethTable.eth[32].ip, "192.168.85.50");
    strcpy(t->ethTable.eth[32].nameIf, "gei1");

    t->ethTable.eth[33].fromTypeNode = E_NODE_PV;
    t->ethTable.eth[33].fromIdNode = 5;
    t->ethTable.eth[33].numIf = 0;
    t->ethTable.eth[33].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[33].toIdNode = 0;
    strcpy(t->ethTable.eth[33].ip, "192.168.72.50");
    strcpy(t->ethTable.eth[33].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_MLMI].fromTypeNode = E_NODE_LMI;
    t->ethTable.eth[ETH_IP_MLMI].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_MLMI].numIf = 0;
    t->ethTable.eth[ETH_IP_MLMI].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_MLMI].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_MLMI].ip, "192.168.72.156");
    strcpy(t->ethTable.eth[ETH_IP_MLMI].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_MLMI_2].fromTypeNode = E_NODE_LMI;
    t->ethTable.eth[ETH_IP_MLMI_2].fromIdNode = 2;
    t->ethTable.eth[ETH_IP_MLMI_2].numIf = 0;
    t->ethTable.eth[ETH_IP_MLMI_2].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_MLMI_2].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_MLMI_2].ip, "192.168.72.241");
    strcpy(t->ethTable.eth[ETH_IP_MLMI_2].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_PW].fromTypeNode = E_NODE_PW;
    t->ethTable.eth[ETH_IP_PW].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_PW].numIf = 0;
    t->ethTable.eth[ETH_IP_PW].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_PW].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_PW].ip, "192.168.72.255");
    strcpy(t->ethTable.eth[ETH_IP_PW].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_KGOO_2].fromTypeNode = E_NODE_KGOO;
    t->ethTable.eth[ETH_IP_KGOO_2].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_KGOO_2].numIf = 0;
    t->ethTable.eth[ETH_IP_KGOO_2].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_KGOO_2].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_KGOO_2].ip, "192.168.72.203");
    strcpy(t->ethTable.eth[ETH_IP_KGOO_2].nameIf, "gei0");

    t->ethTable.eth[ETH_IP_KGOO_RMP_1].fromTypeNode = E_NODE_KGOO;
    t->ethTable.eth[ETH_IP_KGOO_RMP_1].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_KGOO_RMP_1].numIf = 0;
    t->ethTable.eth[ETH_IP_KGOO_RMP_1].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_KGOO_RMP_1].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_KGOO_RMP_1].ip, "192.168.72.199");
    strcpy(t->ethTable.eth[ETH_IP_KGOO_RMP_1].nameIf, "gei0");
    
    t->ethTable.eth[ETH_IP_KGOO_RMP_2].fromTypeNode = E_NODE_KGOO;
    t->ethTable.eth[ETH_IP_KGOO_RMP_2].fromIdNode = 1;
    t->ethTable.eth[ETH_IP_KGOO_RMP_2].numIf = 0;
    t->ethTable.eth[ETH_IP_KGOO_RMP_2].toTypeNode = E_NODE_NONE;
    t->ethTable.eth[ETH_IP_KGOO_RMP_2].toIdNode = 0;
    strcpy(t->ethTable.eth[ETH_IP_KGOO_RMP_2].ip, "192.168.72.198");
    strcpy(t->ethTable.eth[ETH_IP_KGOO_RMP_2].nameIf, "gei0");

    ////////////////
    //! таблица с названиями
    t->nameTable.numNode = 9;
    ///
    t->nameTable.node[0].typeNode = E_NODE_CV;
    t->nameTable.node[0].idNode = 1;
    strcpy(t->nameTable.node[0].name, "CV1");
    strcpy(t->nameTable.node[0].mac, "00-1B-21-A6-CA-B0");

    t->nameTable.node[1].typeNode = E_NODE_CV;
    t->nameTable.node[1].idNode = 2;
    strcpy(t->nameTable.node[1].name, "CV2");
    strcpy(t->nameTable.node[1].mac, "00-1B-21-A6-C8-F0");

    t->nameTable.node[2].typeNode = E_NODE_PV;
    t->nameTable.node[2].idNode = 1;
    strcpy(t->nameTable.node[2].name, "PV1");
    strcpy(t->nameTable.node[2].mac, "00-0B-AB-D5-EE-6F");

    t->nameTable.node[3].typeNode = E_NODE_PV;
    t->nameTable.node[3].idNode = 2;
    strcpy(t->nameTable.node[3].name, "PV2");
    strcpy(t->nameTable.node[3].mac, "00-0B-AB-D5-ED-CD");

    t->nameTable.node[4].typeNode = E_NODE_PV;
    t->nameTable.node[4].idNode = 3;
    strcpy(t->nameTable.node[4].name, "PV3");
    strcpy(t->nameTable.node[4].mac, "00-0B-AB-D5-EE-B1");

    t->nameTable.node[5].typeNode = E_NODE_PV;
    t->nameTable.node[5].idNode = 4;
    strcpy(t->nameTable.node[5].name, "PV4");
    strcpy(t->nameTable.node[5].mac, "00-0B-AB-D4-9A-9A");

    t->nameTable.node[6].typeNode = E_NODE_PV;
    t->nameTable.node[6].idNode = 5;
    strcpy(t->nameTable.node[6].name, "PV5");
    strcpy(t->nameTable.node[6].mac, "C4-00-AD-03-FE-F5");

    t->nameTable.node[7].typeNode = E_NODE_CV;
    t->nameTable.node[7].idNode = 3;
    strcpy(t->nameTable.node[7].name, "CV3");
    strcpy(t->nameTable.node[7].mac, "00-1B-21-A6-CD-FC");

    t->nameTable.node[8].typeNode = E_NODE_PV;
    t->nameTable.node[8].idNode = 9;
    strcpy(t->nameTable.node[8].name, "RP");
    strcpy(t->nameTable.node[8].mac, "00-0B-AB-D5-EE-7F");
*/


    //! настройка каналов
    t->chTable.numCh = MAX_CH_SYS;
    t->chTable.ch[0].typeNode = E_NODE_PV;
    t->chTable.ch[0].idNode = 3;
    t->chTable.ch[0].typeCh = E_CH_RK;
    t->chTable.ch[0].idCh = 0;
    t->chTable.ch[0].setting.numAdapter = 1;
    t->chTable.ch[0].setting.numCh = 12;
    t->chTable.ch[0].setting.ioCh = 0;

    t->chTable.ch[1].typeNode = E_NODE_PV;
    t->chTable.ch[1].idNode = 3;
    t->chTable.ch[1].typeCh = E_CH_RK;
    t->chTable.ch[1].idCh = 0;
    t->chTable.ch[1].setting.numAdapter = 1;
    t->chTable.ch[1].setting.numCh = 12;
    t->chTable.ch[1].setting.ioCh = 0;

    //SVS1_OUT
    arTable.setCh(&(t->chTable.ch[AR_OUT_SVS1]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 16 + 3, 1);
    arTable << 0224 << 0225 << 0223 << 0221 << 0226 << 0230 << 0232 << 0231
            << 0233 << 0335 << 0361 << 0341 << 0363 << 0365;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //RVA098_1
    t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].idNode = 1;
    t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].idCh = 0;
    t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].setting.numCh = LayerMIL::CH_MIL_OSO;//ОСО
    t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_RV098_1_OSO_KOU].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->numWord = 1;
    mil->addr = 26;
    mil->subAddr = 1;
    //RVA098_1
    t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].idNode = 1;
    t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].idCh = 0;
    t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].setting.numCh = LayerMIL::CH_MIL_OSO;//OCO
    t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RV098_1_OSO_OUK].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->numWord = 2;
    mil->addr = 26;
    mil->subAddr = 1;
    //RVA098_2
    t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].idNode = 1;
    t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].idCh = 0;
    t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_RV098_2_NVG_KOU].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->numWord = 1;
    mil->addr = 26;
    mil->subAddr = 1;
    //RVA098_2
    t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].idNode = 1;
    t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].idCh = 0;
    t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RV098_2_NVG_OUK].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->numWord = 2;
    mil->addr = 26;
    mil->subAddr = 1;

    //RVA098_2
    t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].idNode = 3;
    t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].idCh = 0;
    t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_RV098_58_NVG_KOU].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->numWord = 1;
    mil->addr = 26;
    mil->subAddr = 1;
    //RVA098_2
    t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].idNode = 3;
    t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].idCh = 0;
    t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RV098_58_NVG_OUK].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->numWord = 2;
    mil->addr = 26;
    mil->subAddr = 1;

    //AR_RV098_1_OUT
    arTable.setCh(&(t->chTable.ch[AR_OUT_RV098_1]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 16 + 10, 1);
    arTable << 0340;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //AR_RV098_2_OUT
    arTable.setCh(&(t->chTable.ch[AR_OUT_RV098_2]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 16 + 1, 1);
    arTable << 0340;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //SVS2_OUT
    arTable.setCh(&(t->chTable.ch[AR_OUT_SVS2]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 16 + 2, 1);
    arTable << 0224 << 0225 << 0223 << 0221 << 0226 << 0230 << 0232 << 0231
            << 0233 << 0335 << 0361 << 0341 << 0363 << 0365;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM1 (в КСУ)
    arTable.setCh(&(t->chTable.ch[AR_OUT_VIM95_1_KSU]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 0, 1);
    arTable << 0173 << 0174 << 0350 << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM2(в КСУ)
    arTable.setCh(&(t->chTable.ch[AR_OUT_VIM95_2_KSU]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 1, 1);
    arTable << 0173 << 0174 << 0350 << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM1(вход от БЦВМ1)
    arTable.setCh(&(t->chTable.ch[AR_IN_VIM95_1_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 0, 0);
    arTable << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM1(вход от УЦОКС1)
    arTable.setCh(&(t->chTable.ch[AR_IN_VIM95_1_UCOKS]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 6, 0);
    arTable << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM1(вход от БЦВМ2)
    arTable.setCh(&(t->chTable.ch[AR_IN_VIM95_2_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 1, 0);
    arTable << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM2(вход от УЦОКС2)
    arTable.setCh(&(t->chTable.ch[AR_IN_VIM95_2_UCOKS]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 7, 0);
    arTable << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM1(в БЦВМ)
    arTable.setCh(&(t->chTable.ch[AR_OUT_VIM95_1_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 2, 1);
    arTable << 0173 << 0174 << 0350 << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! VIM2(в БЦВМ)
    arTable.setCh(&(t->chTable.ch[AR_OUT_VIM95_2_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 3, 1);
    arTable << 0173 << 0174 << 0350 << 034 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(от БЦВМ1)
    arTable.setCh(&(t->chTable.ch[AR_IN_APDD_1_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 2, 0);
    arTable << 0105 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(от БЦВМ1) для стенда 58
    arTable.setCh(&(t->chTable.ch[AR_IN_APDD_58_1_BCVM]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16 + 2, 0);
    arTable << 0105 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);


    //! АПДД1(от УЦОКС1)
    arTable.setCh(&(t->chTable.ch[AR_IN_APDD_1_UCOKS]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 3, 0);
    arTable << 0105 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(от УЦОКС1) для стенда 58
    arTable.setCh(&(t->chTable.ch[AR_IN_APDD_58_1_UCOKS]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16 + 3, 0);
    arTable << 0105 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);


    //! АПДД2(от БЦВМ1)
    arTable.setCh(&(t->chTable.ch[AR_IN_APDD_2_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 4, 0);
    arTable << 0105 << 033;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД2(от УЦОКС1)
    arTable.setCh(&(t->chTable.ch[AR_IN_APDD_2_UCOKS]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 5, 0);
    arTable << 0105 << 033;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    //!
    //! АПДД1(в БЦВМ1/2 нав. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_1_NAV_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 8, 1);
    arTable << 060 << 076 << 0101 << 0102 << 0103 << 0110 << 0111 << 0112
            << 0120 << 0121 << 0130 << 0133 << 0136 << 0165 << 0166 << 0174
            << 0247 << 0273 << 0355 << 0370;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(в БЦВМ1/2 нав. инф.) стенд 58
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_58_1_NAV_BCVM]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16 + 8, 1);
    arTable << 060 << 076 << 0101 << 0102 << 0103 << 0110 << 0111 << 0112
            << 0120 << 0121 << 0130 << 0133 << 0136 << 0165 << 0166 << 0174
            << 0247 << 0273 << 0355 << 0370;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД2(в БЦВМ1/2 нав. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_2_NAV_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 12, 1);
    arTable << 060 << 076 << 0101 << 0102 << 0103 << 0110 << 0111 << 0112
            << 0120 << 0121 << 0130 << 0133 << 0136 << 0165 << 0166 << 0174
            << 0247 << 0273 << 0355 << 0370;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(в КСУ нав. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_1_NAV_KSU]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 7, 1);
    arTable << 060 << 076 << 0101 << 0102 << 0103 << 0110 << 0111 << 0112
            << 0120 << 0121 << 0130 << 0133 << 0136 << 0165 << 0166 << 0174
            << 0247 << 0273 << 0355 << 0370;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(в КСУ нав. инф.) стенд 58
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_58_1_NAV_KSU]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16+ 7, 1);
    arTable << 060 << 076 << 0101 << 0102 << 0103 << 0110 << 0111 << 0112
            << 0120 << 0121 << 0130 << 0133 << 0136 << 0165 << 0166 << 0174
            << 0247 << 0273 << 0355 << 0370;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД2(в КСУ нав. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_2_NAV_KSU]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 11, 1);
    arTable << 060 << 076 << 0101 << 0102 << 0103 << 0110 << 0111 << 0112
            << 0120 << 0121 << 0130 << 0133 << 0136 << 0165 << 0166 << 0174
            << 0247 << 0273 << 0355 << 0370;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(в КСУ пос. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_1_LAND_KSU]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 9, 1);
    arTable << 0105 << 017 << 033 << 0116 << 0117 << 0126 << 0127 << 0142
            << 0143 << 0144 << 0145 << 0173 << 0174 << 0177 << 0353;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);


    //! АПДД1(в КСУ пос. инф.) стенд 758
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_58_1_LAND_KSU]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16 + 9, 1);
    arTable << 0105 << 017 << 033 << 0116 << 0117 << 0126 << 0127 << 0142
            << 0143 << 0144 << 0145 << 0173 << 0174 << 0177 << 0353 ;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(в БЦВМ1/2 пос. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_1_LAND_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 10, 1);
    arTable << 0105 << 017 << 033 << 0116 << 0117 << 0126 << 0127 << 0142
            << 0143 << 0144 << 0145 << 0173 << 0174 << 0177 << 0353;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД1(в БЦВМ1/2 пос. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_58_1_LAND_BCVM]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16 + 10, 1);
    arTable << 0105 << 017 << 033 << 0116 << 0117 << 0126 << 0127 << 0142
            << 0143 << 0144 << 0145 << 0173 << 0174 << 0177 << 0353;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД2(в БЦВМ1/2 пос. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_2_LAND_BCVM]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 14, 1);
    arTable << 0105 << 017 << 033 << 0116 << 0117 << 0126 << 0127 << 0142
            << 0143 << 0144 << 0145 << 0173 << 0174 << 0177 << 0353 ;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! АПДД2(в КСУ пос. инф.)
    arTable.setCh(&(t->chTable.ch[AR_OUT_APDD_2_LAND_KSU]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 13, 1);
    arTable << 0105 << 017 << 033 << 0116 << 0117 << 0126 << 0127 << 0142
            << 0143 << 0144 << 0145 << 0173 << 0174 << 0177 << 0353;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

//    //! ВСУ(в БКС3)
//    arTable.setCh(&(t->chTable.ch[30]));
//    arTable.setAddr(E_NODE_PV, 1);
//    arTable.setChId(E_CH_AR, 0/*16+2*/, 0);
//    arTable << 0111 << 0266 << 0167 << 0171 << 0272;
//    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
//            LayerArinc::ALWAYS);

    //! КУТР(в БЦВМ)
    arTable.setCh(&(t->chTable.ch[AR_OUT_KUTR_BKS]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 3, 1);
    arTable << 0100 << 0101 << 0102 << 0103 << 0104 << 0105 << 0106 << 0110
            << 0111 << 0112 << 0113 << 0130 << 0131 << 0132 << 0133;
    arTable.setProp(LayerArinc::KBs_12_5, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_KUTR_KSU]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 2, 1);
    arTable << 0100 << 0101 << 0102 << 0103 << 0104 << 0105 << 0106 << 0110
            << 0111 << 0112 << 0113 << 0130 << 0131 << 0132 << 0133;
    arTable.setProp(LayerArinc::KBs_12_5, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    
    arTable.setCh(&(t->chTable.ch[AR_OUT_KUTR_CIMSS]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 1, 1);
    arTable << 0100 << 0101 << 0102 << 0103 << 0104 << 0105 << 0106 << 0110
            << 0111 << 0112 << 0113 << 0130 << 0131 << 0132 << 0133;
    arTable.setProp(LayerArinc::KBs_12_5, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_KUTR_KSS]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 0, 1);
    arTable << 0100 << 0101 << 0102 << 0103 << 0104 << 0105 << 0106 << 0110
            << 0111 << 0112 << 0113 << 0130 << 0131 << 0132 << 0133;
    arTable.setProp(LayerArinc::KBs_12_5, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    //! ВСУ(в УЦОКС1)
    arTable.setCh(&(t->chTable.ch[AR_OUT_VSU_UCOKS]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 16 + 1, 1);
    arTable << 0176 << 0175 << 0316 << 07 << 0244 << 0111 << 0266 << 0165<< 0167 << 0171 << 0272 << 0273 << 0371 << 021;
    //arTable << 0223 << 0224 << 021;// << 0175 << 0316 << 07 << 0244 << 0111 << 0266 << 0165<< 0167 << 0171 << 0272 << 0273 << 0371;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! ВСУ(в БКС3, в УБРП)
    arTable.setCh(&(t->chTable.ch[AR_OUT_VSU_BKS3]));
    arTable.setAddr(E_NODE_PV, 3);
    //arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16 + 0, 1);
    arTable << freqHz(76) << 05 << 0107 << 0303 << 0353 << 0100 << 0102 << 024 << 0111 << 0243 << 0146 << 0227 << 0223 << 0176 << 0307 << 0167 << 0163 << 0371 << 0150 << 0153 << 0311 << 0354 << 0312 << 0226 << 0314 << 0350 << 0110 << 0120 << 02 << 022 << 0
                          << 06 << 0305 << 0304 << 0352 << 0101 << 0103 << 025 << 0266 << 0224 << 0220 << 0222 << 0374 << 0175 << 0300 << 0171 << 0164 << 0166 << 0270 << 016  << 0320 << 0155 << 077 <<  020 <<  0113 << 0124 << 035  << 0121 << 03 << 023 << 0366 << 0
                          << 05 << 0107 << 0303 << 0353 << 0100 << 0102 << 024 << 0111 << 0243 << 0146 << 0227 << 0223 << 0301 << 0172 << 0165 << 0221 << 0225 << 010  << 0316 << 0277 << 0147 << 060 <<  061 <<  062  << 0271 << 0251 << 0170 << 021 << 0367 << 0
                          << 06 << 0305 << 0304 << 0352 << 0101 << 0103 << 025 << 0266 << 0224 << 0220 << 0222 << 0374 << 0244 << 0302 << 0273 << 0272 << 0173 << 0310 << 0317 << 0165 << 0355 << 063 <<  064 <<  065  << 0252 << 0122 << 0362 << 0261 << 0
                          << 05 << 0107 << 0303 << 0353 << 0100 << 0102 << 024 << 0111 << 0243 << 0146 << 0227 << 0223 << 0176 << 0307 << 0167 << 0163 << 0371 << 0150 << 0153 << 07   << 0157 << 0370 << 066 <<  067  << 070  << 0123 << 0125 << 0361 << 0
                          << 06 << 0305 << 0304 << 0352 << 0101 << 0103 << 025 << 0266 << 0224 << 0220 << 0222 << 0374 << 0175 << 0300 << 0171 << 0164 << 0166 << 0270 << 016  << 0306 << 0274 << 0160 << 071 <<  072  << 073  << 0126 << 0127 << 0130 << 0115 << 0
                          << 05 << 0107 << 0303 << 0353 << 0100 << 0102 << 024 << 0111 << 0243 << 0146 << 0227 << 0223 << 0301 << 0172 << 0165 << 0221 << 0225 << 010  << 037  << 0275 << 0161 << 074  << 075 <<  076  << 0253 << 0254 << 0131 << 0356 << 0
                          << 06 << 0305 << 0304 << 0352 << 0101 << 0103 << 025 << 0266 << 0224 << 0220 << 0222 << 0374 << 0244 << 0302 << 0273 << 0272 << 0173 << 0310 << 0162 << 0276 << 0351 << 0104 << 0216<<  0214 << 0145 << 04   << 0360 ;

    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ONE);

    //! БСС(в А1)
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_S_BCVM_2]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 4, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_D_BCVM_1]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 10, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_S_SH1L_1]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 5, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_S_SH2L_3]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 6, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_S_UCOKS_1]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 7, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_S_UBRP]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 8, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_S_CIMSS]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 9, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_D_SH1L_2]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 11, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_D_SH2P_4]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 12, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_D_UCOKS_2]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 13, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    arTable.setCh(&(t->chTable.ch[AR_OUT_BSS117_D_KARP]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 14, 1);
    arTable << 0121 << 0122 << 0123 << 0124 << 0126 << 0127 << 0130 << 0131
            << 0132 << 0134 << 0135;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_IN_IKRL_1_BCVM]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 10, 0);
    arTable << 01 << 02 << 03 << 04 << 05 << 06<< 07 <<010 << 011 << 012
            << 013 << 014 << 015 << 016 << 017 << 020 << 021 << 022 << 023
            << 024 << 025 << 026 << 027 << 030 << 031 << 032 << 033 << 034
            << 035 << 036 << 037 << 040 << 041 << 042 << 043 << 044 << 045
            << 046 << 047 << 050 << 051 << 052 << 053 << 054 << 055 << 056
            << 057 << 060 << 061 << 062 << 063 << 064 << 065 << 066 << 067
            << 070 << 071 << 072 << 073 << 074 << 075 << 076 << 077 << 0100
            << 0101<< 0102<< 0103<< 0104<< 0105<< 0106<< 0107<< 0110 <<0111
            << 0112<< 0113<< 0114<< 0115<< 0116<< 0122<< 0122<< 0123 <<0124
            << 0125<< 0126<< 0127<< 0130<<0131<<0132<<0133<<0117<<0120<<0121
            << 0134<< 0135<< 0136<< 0137<<0140<<0141<<0142<<0143<<0144<<0145
            << 0146<< 0147<< 0150<< 0151<<0152<<0153<<0154<<0155<<0156<<0157
            << 0160<< 0161<< 0162<< 0163;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_IN_IKRL_2_BCVM]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 4, 0);
    arTable << 01 << 02 << 03 << 04 << 05 << 06<< 07 <<010 << 011 << 012
            << 013 << 014 << 015 << 016 << 017 << 020 << 021 << 022 << 023
            << 024 << 025 << 026 << 027 << 030 << 031 << 032 << 033 << 034
            << 035 << 036 << 037 << 040 << 041 << 042 << 043 << 044 << 045
            << 046 << 047 << 050 << 051 << 052 << 053 << 054 << 055 << 056
            << 057 << 060 << 061 << 062 << 063 << 064 << 065 << 066 << 067
            << 070 << 071 << 072 << 073 << 074 << 075 << 076 << 077 << 0100
            << 0101<< 0102<< 0103<< 0104<< 0105<< 0106<< 0107<< 0110<< 0111
            << 0112<< 0113<< 0114<< 0115<< 0116<< 0122<< 0123<< 0124
            << 0125<< 0126<< 0127<< 0130<< 0131<< 0132<< 0133<< 0117<< 0120 << 0121
            << 0134<< 0135<< 0136<< 0137<< 0140<< 0141<< 0142<< 0143<< 0144 << 0145
            << 0146<< 0147<< 0150<< 0151<< 0152<< 0153<< 0154<< 0155<< 0156 << 0157
            << 0160<< 0161<< 0162<< 0163;

    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);
    //! прием данны от БКС1
    arTable.setCh(&(t->chTable.ch[AR_IN_IKRL_1_BKS1]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 12, 0);//!
    arTable << 0100 << 0101<< 0102<< 0103<< 0104<< 0105<< 0106<< 0107<< 0110
            <<0111  << 0112<< 0113<< 0114<< 0115<< 0116<< 0122<< 0123
            <<0124  << 0125<< 0126<< 0127<< 0130;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3, LayerArinc::ALWAYS);

    //! прием данны от БКС3
    arTable.setCh(&(t->chTable.ch[AR_IN_IKRL_1_BKS3]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 13, 0);//!
    arTable << 0100 << 0101<< 0102<< 0103<< 0104<< 0105<< 0106<< 0107<< 0110
            <<0111  << 0112<< 0113<< 0114<< 0115<< 0116<< 0122<< 0123
            <<0124  << 0125<< 0126<< 0127<< 0130;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3, LayerArinc::ALWAYS);

    //! прием данны от БКС3
       arTable.setCh(&(t->chTable.ch[AR_IN_IKRL_2_BKS2]));
       arTable.setAddr(E_NODE_PV, 4);
       arTable.setChId(E_CH_AR, 2, 0);//!
       arTable << 0100 << 0101<< 0102<< 0103<< 0104<< 0105<< 0106<< 0107<< 0110
               << 0111 << 0112<< 0113<< 0114<< 0115<< 0116<< 0122<< 0123
               << 0124 << 0125<< 0126<< 0127;
       arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3, LayerArinc::ALWAYS);

       arTable.setCh(&(t->chTable.ch[AR_IN_IKRL_2_BKS4]));
         arTable.setAddr(E_NODE_PV, 4);
         arTable.setChId(E_CH_AR, 3, 0);//!
         arTable << 0100 << 0101<< 0102<< 0103<< 0104 << 0105 << 0106 << 0107 << 0110
                 << 0111 << 0112<< 0113<< 0114<< 0115 << 0116 << 0122 << 0123
                 << 0124 << 0125<< 0126<< 0127<< 0130 << 0131 << 0132 << 0133
                 << 0134 << 0135<< 0136<< 0137<< 0140 << 0141 <<0142 ;
         arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3, LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_IKRL_1_1_BCVM]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 0, 1);
    arTable << 01  << 02  << 03  <<  04  <<  06  << 014  <<  015 <<  033 <<  025 <<  026  <<  030
            << 027 << 05  << 07  <<  010 <<  011 << 012  <<  013 <<  016 <<  017 <<  020  <<  021
            << 022 << 023 << 024;

    /*arTable << 01  << 02  <<  03  <<  04  <<  05  <<  06  <<  07  <<  010 <<  011 <<  012 <<  013 <<  014
            << 015 << 016 <<  017 <<  020 <<  021 <<  022 <<  023 <<  024 <<  025 <<  026 <<  027 <<  030
            << 031 << 032 <<  033 <<  034 <<  035 <<  036 <<  037 <<  040 <<  041 <<  042 <<  043 <<  044;*/
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_IKRL_1_2_BCVM]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 3, 1);
    arTable << 01  << 02  << 03  <<  04  <<  06  << 014  <<  015 <<  033 <<  025 <<  026  <<  030
            << 027 << 05  << 07  <<  010 <<  011 << 012  <<  013 <<  016 <<  017 <<  020  <<  021
            << 022 << 023 << 024;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_IKRL_2_1_BCVM]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 1, 1);
    arTable << 01  << 02  << 03  <<  04  <<  06  << 014  <<  015 <<  033 <<  025 <<  026  <<  030
            << 027 << 05  << 07  <<  010 <<  011 << 012  <<  013 <<  016 <<  017 <<  020  <<  021
            << 022 << 023 << 024;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_IKRL_2_2_BCVM]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 2, 1);
    arTable << 01  << 02  << 03  <<  04  <<  06  << 014  <<  015 <<  033 <<  025 <<  026  <<  030
            << 027 << 05  << 07  <<  010 <<  011 << 012  <<  013 <<  016 <<  017 <<  020  <<  021
            << 022 << 023 << 024;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_IKRL_1_KSU_1]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 5, 1);
    arTable << 034 << 05 << 0171 << 0172 << 026 << 055 << 0121 << 057 << 0220 << 06 << 07 << 01 << 02 << 03 << 04 << 010 << 011;

    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

//    arTable.setCh(&(t->chTable.ch[AR_OUT_IKRL_1_KSU_3]));
//    arTable.setAddr(E_NODE_PV, 4);
//    arTable.setChId(E_CH_AR, 2, 1);
//    arTable << 034 << 05 << 0171 << 0172 << 026 << 055 << 0121 << 057 << 0220 << 06 << 07 << 01 << 02 << 03 << 04;

//    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
//            LayerArinc::ALWAYS);







    //! ЦИМСС НВГ 1 ... 13
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 1;
    mil->numWord = 32;

    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 1;
    mil->numWord = 32;

    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 2;
    mil->numWord = 32;

    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 2;
    mil->numWord = 32;

    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 3;
    mil->numWord = 32;

    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 3;
    mil->numWord = 32;

    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 4;
    mil->numWord = 13;

    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 4;
    mil->numWord = 13;

    //! приемная часть от БЦВМ

    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 1;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 2;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 3;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA4].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA4].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA4].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA4].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_NVG_SA4].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 4;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA5].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 5;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA6].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 6;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA7].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 7;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA8].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 8;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA9].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 9;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA10].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 10;
    mil->numWord = 3;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA11].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 11;
    mil->numWord = 4;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_NVG_SA12].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 4;
    mil->numWord = 13;



    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA1].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 1;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 2;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA3].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 3;
    mil->numWord = 32;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].idNode = 1;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_CIMSS_A_KSS_SA4].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 4;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA5].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 5;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA6].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 6;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA7].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 7;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA8].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 8;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA9].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 9;
    mil->numWord = 32;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA10].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 10;
    mil->numWord = 3;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA11].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 11;
    mil->numWord = 4;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].idNode = 1;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].idCh = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].setting.numCh = LayerMIL::CH_MIL_KSS;
    t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_CIMSS_A_KSS_SA12].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 5;
    mil->subAddr = 4;
    mil->numWord = 32;

    //! БИНС1 1 подадрес
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 1;
    mil->numWord = 26;

    //! БИНС1 2 подадрес
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 2;
    mil->numWord = 21;
    //! БИНС1 3 адрес
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_SP2_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 3;
    mil->numWord = 17;

    //! БИНС1 4 адрес
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_SP2_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 4;
    mil->numWord = 31;

    //! БИНС1 5 адрес
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_SP2_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 5;
    mil->numWord = 16;

    //! БИНС1 1 адрес
    t->chTable.ch[MIL_IN_BINS1_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS1_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 27;
    mil->subAddr = 1;
    mil->numWord = 9;

    //! БИНС1 2 адрес
    t->chTable.ch[MIL_IN_BINS1_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BINS1_SP2_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS1_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 27;
    mil->subAddr = 2;
    mil->numWord = 9;

    /////////////////////////////////////////////////////////////////
    //! БИНС2 1 подадрес
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 1;
    mil->numWord = 26;

    //! БИНС1 2 подадрес
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 2;
    mil->numWord = 21;
    //! БИНС1 3 адрес
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_SP2_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 3;
    mil->numWord = 17;

    //! БИНС1 4 адрес
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_SP2_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 4;
    mil->numWord = 31;

    //! БИНС1 5 адрес
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_SP2_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 5;
    mil->numWord = 16;

    //! БИНС1 1 адрес
    t->chTable.ch[MIL_IN_BINS2_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS2_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 25;
    mil->subAddr = 1;
    mil->numWord = 9;

    //! БИНС1 2 адрес
    t->chTable.ch[MIL_IN_BINS2_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BINS2_SP2_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS2_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 25;
    mil->subAddr = 2;
    mil->numWord = 9;
    ////////////////////////////////////////////////////////////////
    //! БИНС3 1 подадрес
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS3_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 1;
    mil->numWord = 26;

    //! БИНС3 2 подадрес
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS3_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 2;
    mil->numWord = 21;
    //! БИНС3 3 адрес
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS3_SP2_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 3;
    mil->numWord = 17;

    //! БИНС3 4 адрес
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS3_SP2_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 4;
    mil->numWord = 31;

    //! БИНС3 5 адрес
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].idNode = 1;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS3_SP2_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 5;
    mil->numWord = 16;

    //! БИНС3 1 адрес
    t->chTable.ch[MIL_IN_BINS3_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS3_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 28;
    mil->subAddr = 1;
    mil->numWord = 9;

    //! БИНС3 2 адрес
    t->chTable.ch[MIL_IN_BINS3_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BINS3_SP2_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS3_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 28;
    mil->subAddr = 2;
    mil->numWord = 9;
    ////////////////////////////////////////////////////////////
    //! КУТР Р
    t->chTable.ch[MIL_IN_KUTR_R_OSO].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_KUTR_R_OSO].idNode = 1;
    t->chTable.ch[MIL_IN_KUTR_R_OSO].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_KUTR_R_OSO].idCh = 0;
    t->chTable.ch[MIL_IN_KUTR_R_OSO].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_KUTR_R_OSO].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_KUTR_R_OSO].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KUTR_R_OSO].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 7;
    mil->subAddr = 3;
    mil->numWord = 3;

    //! КУТР Р
    t->chTable.ch[MIL_OUT_KUTR_R_OSO].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR_R_OSO].idNode = 1;
    t->chTable.ch[MIL_OUT_KUTR_R_OSO].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR_R_OSO].idCh = 0;
    t->chTable.ch[MIL_OUT_KUTR_R_OSO].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KUTR_R_OSO].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_KUTR_R_OSO].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR_R_OSO].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 7;
    mil->subAddr = 2;
    mil->numWord = 15;

    //! КУТР А
    t->chTable.ch[MIL_IN_KUTR_A_NVG].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_KUTR_A_NVG].idNode = 1;
    t->chTable.ch[MIL_IN_KUTR_A_NVG].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_KUTR_A_NVG].idCh = 0;
    t->chTable.ch[MIL_IN_KUTR_A_NVG].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_KUTR_A_NVG].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_KUTR_A_NVG].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KUTR_A_NVG].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 8;
    mil->subAddr = 3;
    mil->numWord = 3;

    //! КУТР А
    t->chTable.ch[MIL_OUT_KUTR_A_NVG].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR_A_NVG].idNode = 1;
    t->chTable.ch[MIL_OUT_KUTR_A_NVG].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR_A_NVG].idCh = 0;
    t->chTable.ch[MIL_OUT_KUTR_A_NVG].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KUTR_A_NVG].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_KUTR_A_NVG].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR_A_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 8;
    mil->subAddr = 2;
    mil->numWord = 15;

    //! КСУ НВГ 17
    t->chTable.ch[MIL_OUT_KSU_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 11;
    mil->subAddr = 17;
    mil->numWord = 25;

    //! КСУ НВГ 18
    t->chTable.ch[MIL_OUT_KSU_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 11;
    mil->subAddr = 18;
    mil->numWord = 26;

    //! КСУ НВГ 19
    t->chTable.ch[MIL_OUT_KSU_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_KSU_NVG_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 11;
    mil->subAddr = 19;
    mil->numWord = 22;

    //! КСУ НВГ 21
    t->chTable.ch[MIL_IN_KSU_NVG_IN1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU_NVG_IN1].idNode = 1;
    t->chTable.ch[MIL_IN_KSU_NVG_IN1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU_NVG_IN1].idCh = 0;
    t->chTable.ch[MIL_IN_KSU_NVG_IN1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_KSU_NVG_IN1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_KSU_NVG_IN1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU_NVG_IN1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 11;
    mil->subAddr = 21;
    mil->numWord = 30;

    //! КСУ НВГ 22
    t->chTable.ch[MIL_IN_KSU_NVG_IN2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU_NVG_IN2].idNode = 1;
    t->chTable.ch[MIL_IN_KSU_NVG_IN2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU_NVG_IN2].idCh = 0;
    t->chTable.ch[MIL_IN_KSU_NVG_IN2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_KSU_NVG_IN2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_KSU_NVG_IN2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU_NVG_IN2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 11;
    mil->subAddr = 22;
    mil->numWord = 17;

    //        //! КСУ НВГ 23
    //        t->chTable.ch[65].typeNode           = E_NODE_CV;
    //        t->chTable.ch[65].idNode             = 1;
    //        t->chTable.ch[65].typeCh             = E_CH_MIL;
    //        t->chTable.ch[65].idCh               = 0;
    //        t->chTable.ch[65].setting.numAdapter = 0;
    //        t->chTable.ch[65].setting.numCh      = LayerMIL::CH_MIL_NVG;
    //        t->chTable.ch[65].setting.ioCh       = 0;
    //        mil = (TDesMIL*)(t->chTable.ch[65].desData);
    //        mil->typeTrans = LayerMIL::KOU;
    //        mil->addr       = 11;
    //        mil->subAddr    = 23;
    //        mil->numWord    = 21;

    //! КСУ ОСО 17
    t->chTable.ch[MIL_OUT_KSU_OSO_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU_OSO_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 21;
    mil->subAddr = 17;
    mil->numWord = 25;

    //! КСУ ОСО 18
    t->chTable.ch[MIL_OUT_KSU_OSO_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU_OSO_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 21;
    mil->subAddr = 18;
    mil->numWord = 26;

    //! КСУ ОСО 19
    t->chTable.ch[MIL_OUT_KSU_OSO_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA3].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_KSU_OSO_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU_OSO_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 21;
    mil->subAddr = 19;
    mil->numWord = 22;

    //! КСУ ОСО 21
    t->chTable.ch[MIL_IN_KSU_OSO_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU_OSO_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_KSU_OSO_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU_OSO_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_KSU_OSO_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_KSU_OSO_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_KSU_OSO_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU_OSO_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 21;
    mil->subAddr = 21;
    mil->numWord = 30;

    //! КСУ ОСО 22
    t->chTable.ch[MIL_IN_KSU_OSO_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU_OSO_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_KSU_OSO_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU_OSO_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_KSU_OSO_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_KSU_OSO_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_KSU_OSO_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU_OSO_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 21;
    mil->subAddr = 22;
    mil->numWord = 17;

    ///////////////////////////////////////////////////////////////////////////////////////
    //! КСУ58 НВГ1 17
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG1_SA1].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 11;
    mil->subAddr    = 17;
    mil->numWord    = 31;

    //! КСУ58 НВГ1 18
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG1_SA2].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 11;
    mil->subAddr    = 18;
    mil->numWord    = 31;

    //! КСУ58 НВГ1 19
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG1_SA3].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 11;
    mil->subAddr    = 19;
    mil->numWord    = 31;

    //! КСУ58 НВГ1 20
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG1_SA4].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 11;
    mil->subAddr    = 20;
    mil->numWord    = 24;

    //! КСУ58 НВГ1 21
    t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].idNode                = 3;
    t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].idCh                  = 0;
    t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].setting.ioCh          = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU58_NVG1_IN1].desData);
    mil->typeTrans  = LayerMIL::KOU;
    mil->addr       = 11;
    mil->subAddr    = 21;
    mil->numWord    = 31;

    ////////////////////////////////////////////////////////////////////////////////////////////
    //! КСУ58 НВГ1 17
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG2_SA1].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 12;
    mil->subAddr    = 17;
    mil->numWord    = 31;

    //! КСУ58 НВГ1 18
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG2_SA2].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 12;
    mil->subAddr    = 18;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 19
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG2_SA3].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 12;
    mil->subAddr    = 19;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 20
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_NVG2_SA4].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 12;
    mil->subAddr    = 20;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 21
    t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].idNode                = 3;
    t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].idCh                  = 0;
    t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].setting.numCh         = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].setting.ioCh          = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU58_NVG2_IN1].desData);
    mil->typeTrans  = LayerMIL::KOU;
    mil->addr       = 12;
    mil->subAddr    = 21;
    mil->numWord    = 1;


    ///////////////////////////////////////////////////////////////////////////////////////
    //! КСУ58 НВГ1 17
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP1_SA1].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 13;
    mil->subAddr    = 17;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 18
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP1_SA2].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 13;
    mil->subAddr    = 18;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 19
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP1_SA3].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 13;
    mil->subAddr    = 19;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 20
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP1_SA4].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 13;
    mil->subAddr    = 20;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 21
    t->chTable.ch[MIL_IN_KSU58_BP1_IN1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU58_BP1_IN1].idNode                = 3;
    t->chTable.ch[MIL_IN_KSU58_BP1_IN1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU58_BP1_IN1].idCh                  = 0;
    t->chTable.ch[MIL_IN_KSU58_BP1_IN1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_IN_KSU58_BP1_IN1].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_IN_KSU58_BP1_IN1].setting.ioCh          = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU58_BP1_IN1].desData);
    mil->typeTrans  = LayerMIL::KOU;
    mil->addr       = 13;
    mil->subAddr    = 21;
    mil->numWord    = 1;

    ////////////////////////////////////////////////////////////////////////////////////////////
    //! КСУ58 НВГ1 17
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP2_SA1].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 14;
    mil->subAddr    = 17;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 18
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP2_SA2].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 14;
    mil->subAddr    = 18;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 19
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP2_SA3].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 14;
    mil->subAddr    = 19;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 20
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].idNode                = 3;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].idCh                  = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].setting.numAdapter    = 0;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].setting.ioCh          = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KSU58_BP2_SA4].desData);
    mil->typeTrans  = LayerMIL::OUK;
    mil->addr       = 14;
    mil->subAddr    = 20;
    mil->numWord    = 1;

    //! КСУ58 НВГ1 21
    t->chTable.ch[MIL_IN_KSU58_BP2_IN1].typeNode              = E_NODE_CV;
    t->chTable.ch[MIL_IN_KSU58_BP2_IN1].idNode                = 3;
    t->chTable.ch[MIL_IN_KSU58_BP2_IN1].typeCh                = E_CH_MIL;
    t->chTable.ch[MIL_IN_KSU58_BP2_IN1].idCh                  = 0;
    t->chTable.ch[MIL_IN_KSU58_BP2_IN1].setting.numAdapter    = 0;
    t->chTable.ch[MIL_IN_KSU58_BP2_IN1].setting.numCh         = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_IN_KSU58_BP2_IN1].setting.ioCh          = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_KSU58_BP2_IN1].desData);
    mil->typeTrans  = LayerMIL::KOU;
    mil->addr       = 14;
    mil->subAddr    = 21;
    mil->numWord    = 1;
//    MIL_OUT_KSU58_NVG1_SA1          = 462,
//    MIL_OUT_KSU58_NVG1_SA2          = 463,
//    MIL_OUT_KSU58_NVG1_SA3          = 464,
//    MIL_OUT_KSU58_NVG1_SA4          = 465,

//    MIL_IN_KSU58_NVG1_IN1           = 466,
//    MIL_IN_KSU58_NVG2_IN1           = 467,
//    MIL_IN_KSU58_BP1_IN1            = 468,
//    MIL_IN_KSU58_BP2_IN1            = 469,

//    MIL_OUT_KSU58_NVG2_SA1          = 470,
//    MIL_OUT_KSU58_NVG2_SA2          = 471,
//    MIL_OUT_KSU58_NVG2_SA3          = 472,
//    MIL_OUT_KSU58_NVG2_SA4          = 473,

//    MIL_OUT_KSU58_BP1_SA1           = 474,
//    MIL_OUT_KSU58_BP1_SA2           = 475,
//    MIL_OUT_KSU58_BP1_SA3           = 476,
//    MIL_OUT_KSU58_BP1_SA4           = 477,

//    MIL_OUT_KSU58_BP2_SA1           = 478,
//    MIL_OUT_KSU58_BP2_SA2           = 479,
//    MIL_OUT_KSU58_BP2_SA3           = 480,
//    MIL_OUT_KSU58_BP2_SA4           = 481

//    //! КСУ ОСО 23
//    t->chTable.ch[71].typeNode = E_NODE_CV;
//    t->chTable.ch[71].idNode = 1;
//    t->chTable.ch[71].typeCh = E_CH_MIL;
//    t->chTable.ch[71].idCh = 0;
//    t->chTable.ch[71].setting.numAdapter = 0;
//    t->chTable.ch[71].setting.numCh = LayerMIL::CH_MIL_OSO;
//    t->chTable.ch[71].setting.ioCh = 0;
//    mil = (TDesMIL*) (t->chTable.ch[71].desData);
//    mil->typeTrans = LayerMIL::KOU;
//    mil->addr = 21;
//    mil->subAddr = 23;
//    mil->numWord = 21;

    //! БСС С ОСО
    t->chTable.ch[MIL_OUT_BSS_OSO_S].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BSS_OSO_S].idNode = 1;
    t->chTable.ch[MIL_OUT_BSS_OSO_S].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BSS_OSO_S].idCh = 0;
    t->chTable.ch[MIL_OUT_BSS_OSO_S].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BSS_OSO_S].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BSS_OSO_S].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BSS_OSO_S].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 16;
    mil->subAddr = 1;
    mil->numWord = 11;

    //! БСС С ОСО
    t->chTable.ch[MIL_IN_BSS_OSO_S].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BSS_OSO_S].idNode = 1;
    t->chTable.ch[MIL_IN_BSS_OSO_S].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BSS_OSO_S].idCh = 0;
    t->chTable.ch[MIL_IN_BSS_OSO_S].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BSS_OSO_S].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BSS_OSO_S].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BSS_OSO_S].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 16;
    mil->subAddr = 1;
    mil->numWord = 16;

    //! БСС Д ОСО
    t->chTable.ch[MIL_OUT_BSS_OSO_D].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BSS_OSO_D].idNode = 1;
    t->chTable.ch[MIL_OUT_BSS_OSO_D].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BSS_OSO_D].idCh = 0;
    t->chTable.ch[MIL_OUT_BSS_OSO_D].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BSS_OSO_D].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BSS_OSO_D].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BSS_OSO_D].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 17;
    mil->subAddr = 1;
    mil->numWord = 11;

    //! БСС Д ОСО
    t->chTable.ch[MIL_IN_BSS_OSO_D].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BSS_OSO_D].idNode = 1;
    t->chTable.ch[MIL_IN_BSS_OSO_D].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BSS_OSO_D].idCh = 0;
    t->chTable.ch[MIL_IN_BSS_OSO_D].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BSS_OSO_D].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BSS_OSO_D].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BSS_OSO_D].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 17;
    mil->subAddr = 1;
    mil->numWord = 16;

//    //! БСС Д ОСО
//    t->chTable.ch[MIL_IN_BSS_OSO_D].typeNode = E_NODE_EISA;
//    t->chTable.ch[MIL_IN_BSS_OSO_D].idNode = 0;
//    t->chTable.ch[MIL_IN_BSS_OSO_D].typeCh = E_CH_RK;
//    t->chTable.ch[P_BINS_SNS_B1].idCh = 0;
//    t->chTable.ch[P_BINS_SNS_B1].setting.numAdapter = 0;
//    t->chTable.ch[P_BINS_SNS_B1].setting.numCh = 0;
//    t->chTable.ch[P_BINS_SNS_B1].setting.ioCh = 1;

    //!КВ ЗВП левой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_1].setting.numCh = 48 + 0;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_1].setting.ioCh = 1;
    //КВ ЗВП левой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_2].setting.numCh = 2* 48 + 23;
    t->chTable.ch[SUVSH_KV_ZVP_LOOSH_2].setting.ioCh = 1;
    //КВ ЗВП ПОШ(1)
    t->chTable.ch[SUVSH_KV_ZVP_POSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_1].setting.numCh = 48 + 1;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_1].setting.ioCh = 1;
    //КВ ЗВП ПОШ(2)
    t->chTable.ch[SUVSH_KV_ZVP_POSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_2].setting.numCh = 2 * 48 + 24;
    t->chTable.ch[SUVSH_KV_ZVP_POSH_2].setting.ioCh = 1;
    //КВ ЗВП правой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_1].setting.numCh = 48 + 2;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_1].setting.ioCh = 1;
    //КВ ЗВП правой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_2].setting.numCh = 2 * 48 + 25;
    t->chTable.ch[SUVSH_KV_ZVP_ROOSH_2].setting.ioCh = 1;
    //КВ ЗУП створки левой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_1].setting.numCh = 48 + 3;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_1].setting.ioCh = 1;
    //КВ ЗУП створки левой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_2].setting.numCh = 2 * 48 + 26;
    t->chTable.ch[SUVSH_KV_ZUP_STLOOSH_2].setting.ioCh = 1;
    //КВ ЗУП створки правой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_1].setting.numCh = 48 + 4;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_1].setting.ioCh = 1;
    //КВ ЗУП створки правой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_2].setting.numCh = 2 * 48 + 27;
    t->chTable.ch[SUVSH_KV_ZUP_STROOSH_2].setting.ioCh = 1;

    //КВ ЗУП створки ПОШ(1)
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_1].setting.numCh = 48 + 5;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_1].setting.ioCh = 1;

    //КВ ЗУП створки ПОШ(2)
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_2].setting.numCh = 2* 48 + 28;
    t->chTable.ch[SUVSH_KV_ZUP_STPOSH_2].setting.ioCh = 1;

    //КВ ЗУП ПОШ(1)
    t->chTable.ch[SUVSH_KV_ZUP_POSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_1].setting.numCh = 48 + 6;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_1].setting.ioCh = 1;

    //КВ ЗУП ПОШ(2)
    t->chTable.ch[SUVSH_KV_ZUP_POSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_2].setting.numCh = 2 * 48 + 29;
    t->chTable.ch[SUVSH_KV_ZUP_POSH_2].setting.ioCh = 1;

    //КВ ЗУП левой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_1].setting.numCh = 48 + 7;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_1].setting.ioCh = 1;

    //КВ ЗУП левой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_2].setting.numCh = 2 * 48 + 30;
    t->chTable.ch[SUVSH_KV_ZUP_LOOSH_2].setting.ioCh = 1;

    //КВ ЗУП правой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_1].setting.numCh = 48 + 8;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_1].setting.ioCh = 1;

    //КВ ЗУП правой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_2].setting.numCh = 2 * 48 + 31;
    t->chTable.ch[SUVSH_KV_ZUP_ROOSH_2].setting.ioCh = 1;

    //КВ ЗВП створки правой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_1].setting.numCh = 48 + 9;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_1].setting.ioCh = 1;

    //КВ ЗВП створки правой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_2].setting.numCh = 2 * 48 + 32;
    t->chTable.ch[SUVSH_KV_ZVP_STROOSH_2].setting.ioCh = 1;

    //КВ ЗВП створки левой ООШ(1)
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_1].setting.numCh = 48 + 10;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_1].setting.ioCh = 1;

    //КВ ЗВП створки левой ООШ(2)
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_2].setting.numCh = 2 * 48 + 33;
    t->chTable.ch[SUVSH_KV_ZVP_STLOOSH_2].setting.ioCh = 1;

    //КВ ЗВП створки ПОШ(1)
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_1].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_1].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_1].setting.numCh = 48 + 11;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_1].setting.ioCh = 1;

    //КВ ЗВП створки ПОШ(2)
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_2].idNode = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_2].idCh = 0;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_2].setting.numCh = 2 * 48 + 34;
    t->chTable.ch[SUVSH_KV_ZVP_STPOSH_2].setting.ioCh = 1;

    //Контроль электроцепей шасси (1)
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_1].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_1].idNode = 1;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_1].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_1].idCh = 0;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_1].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_1].setting.numCh = 48 + 12;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_1].setting.ioCh = 1;

    t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_1].typeNode = E_NODE_PV;
        t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_1].idNode = 1;
        t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_1].typeCh = E_CH_RK;
        t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_1].idCh = 0;
        t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_1].setting.numAdapter = 1;
        t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_1].setting.numCh = 48 + 46;
        t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_1].setting.ioCh = 1;

        t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_2].typeNode = E_NODE_PV;
            t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_2].idNode = 1;
            t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_2].typeCh = E_CH_RK;
            t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_2].idCh = 0;
            t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_2].setting.numAdapter = 1;
            t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_2].setting.numCh = 48 + 47;
            t->chTable.ch[SUVSH_RUCH_ST_TORM_OTKL_2].setting.ioCh = 1;

    //Контроль электроцепей шасси (2)
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_2].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_2].idNode = 1;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_2].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_2].idCh = 0;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_2].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_2].setting.numCh = 2 * 48 + 35; //48 + 12;
    t->chTable.ch[SUVSH_KONTR_CPEJ_SHASSI_2].setting.ioCh = 1;
    //Отключение охлаждения колес
    t->chTable.ch[SUVSH_OTKL_OHL_KOLES].typeNode = E_NODE_PV;
    t->chTable.ch[SUVSH_OTKL_OHL_KOLES].idNode = 1;
    t->chTable.ch[SUVSH_OTKL_OHL_KOLES].typeCh = E_CH_RK;
    t->chTable.ch[SUVSH_OTKL_OHL_KOLES].idCh = 0;
    t->chTable.ch[SUVSH_OTKL_OHL_KOLES].setting.numAdapter = 1;
    t->chTable.ch[SUVSH_OTKL_OHL_KOLES].setting.numCh = 48 + 45;
    t->chTable.ch[SUVSH_OTKL_OHL_KOLES].setting.ioCh = 1;

    //SVS3_OUT
    arTable.setCh(&(t->chTable.ch[AR_OUT_SVS3]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 16 + 4, 1);
    arTable << 0224 << 0225 << 0223 << 0221 << 0226 << 0230 << 0232 << 0231
            << 0233 << 0335 << 0361 << 0341 << 0363 << 0365;
    arTable.setProp(LayerArinc::KBs_50, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //КВ ШВ 1
    t->chTable.ch[KSU_KV_SHV1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_SHV1].idNode = 3;
    t->chTable.ch[KSU_KV_SHV1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_SHV1].idCh = 0;
    t->chTable.ch[KSU_KV_SHV1].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_SHV1].setting.numCh = 21;
    t->chTable.ch[KSU_KV_SHV1].setting.ioCh = 1;

    //КВ ШВ 2
    t->chTable.ch[KSU_KV_SHV2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_SHV2].idNode = 3;
    t->chTable.ch[KSU_KV_SHV2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_SHV2].idCh = 0;
    t->chTable.ch[KSU_KV_SHV2].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_SHV2].setting.numCh = 22;
    t->chTable.ch[KSU_KV_SHV2].setting.ioCh = 1;

    //КВ ШВ 3
    t->chTable.ch[KSU_KV_SHV3].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_SHV3].idNode = 3;
    t->chTable.ch[KSU_KV_SHV3].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_SHV3].idCh = 0;
    t->chTable.ch[KSU_KV_SHV3].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_SHV3].setting.numCh = 23;
    t->chTable.ch[KSU_KV_SHV3].setting.ioCh = 1;

    //КВ ШВ 4
    t->chTable.ch[KSU_KV_SHV4].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_SHV4].idNode = 3;
    t->chTable.ch[KSU_KV_SHV4].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_SHV4].idCh = 0;
    t->chTable.ch[KSU_KV_SHV4].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_SHV4].setting.numCh = 24;
    t->chTable.ch[KSU_KV_SHV4].setting.ioCh = 1;

    //КВ ОПС 1
    t->chTable.ch[KSU_KV_OPC1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OPC1].idNode = 3;
    t->chTable.ch[KSU_KV_OPC1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OPC1].idCh = 0;
    t->chTable.ch[KSU_KV_OPC1].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OPC1].setting.numCh = 25;
    t->chTable.ch[KSU_KV_OPC1].setting.ioCh = 1;

    //КВ ОПС 2
    t->chTable.ch[KSU_KV_OPC2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OPC2].idNode = 3;
    t->chTable.ch[KSU_KV_OPC2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OPC2].idCh = 0;
    t->chTable.ch[KSU_KV_OPC2].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OPC2].setting.numCh = 26;
    t->chTable.ch[KSU_KV_OPC2].setting.ioCh = 1;

    //КВ ОПС 3
    t->chTable.ch[KSU_KV_OPC3].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OPC3].idNode = 3;
    t->chTable.ch[KSU_KV_OPC3].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OPC3].idCh = 0;
    t->chTable.ch[KSU_KV_OPC3].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OPC3].setting.numCh = 27;
    t->chTable.ch[KSU_KV_OPC3].setting.ioCh = 1;

    //КВ ОПС 4
    t->chTable.ch[KSU_KV_OPC4].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OPC4].idNode = 3;
    t->chTable.ch[KSU_KV_OPC4].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OPC4].idCh = 0;
    t->chTable.ch[KSU_KV_OPC4].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OPC4].setting.numCh = 28;
    t->chTable.ch[KSU_KV_OPC4].setting.ioCh = 1;

    //КВ ОГС1 лев
    t->chTable.ch[KSU_KV_OGS_L1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OGS_L1].idNode = 3;
    t->chTable.ch[KSU_KV_OGS_L1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OGS_L1].idCh = 0;
    t->chTable.ch[KSU_KV_OGS_L1].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OGS_L1].setting.numCh = 29;
    t->chTable.ch[KSU_KV_OGS_L1].setting.ioCh = 1;

    //КВ ОГС2 лев
    t->chTable.ch[KSU_KV_OGS_L2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OGS_L2].idNode = 3;
    t->chTable.ch[KSU_KV_OGS_L2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OGS_L2].idCh = 0;
    t->chTable.ch[KSU_KV_OGS_L2].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OGS_L2].setting.numCh = 30;
    t->chTable.ch[KSU_KV_OGS_L2].setting.ioCh = 1;

    //КВ ОГС1 прав
    t->chTable.ch[KSU_KV_OGS_P1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OGS_P1].idNode = 3;
    t->chTable.ch[KSU_KV_OGS_P1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OGS_P1].idCh = 0;
    t->chTable.ch[KSU_KV_OGS_P1].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OGS_P1].setting.numCh = 31;
    t->chTable.ch[KSU_KV_OGS_P1].setting.ioCh = 1;

    //КВ ОГС2 прав
    t->chTable.ch[KSU_KV_OGS_P2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_KV_OGS_P2].idNode = 3;
    t->chTable.ch[KSU_KV_OGS_P2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_KV_OGS_P2].idCh = 0;
    t->chTable.ch[KSU_KV_OGS_P2].setting.numAdapter = 1;
    t->chTable.ch[KSU_KV_OGS_P2].setting.numCh = 32;
    t->chTable.ch[KSU_KV_OGS_P2].setting.ioCh = 1;

    //АварПит1
    t->chTable.ch[KSU_AVAR_PIT1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AVAR_PIT1].idNode = 3;
    t->chTable.ch[KSU_AVAR_PIT1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AVAR_PIT1].idCh = 0;
    t->chTable.ch[KSU_AVAR_PIT1].setting.numAdapter = 1;
    t->chTable.ch[KSU_AVAR_PIT1].setting.numCh = 33;
    t->chTable.ch[KSU_AVAR_PIT1].setting.ioCh = 1;

    //АварПит2
    t->chTable.ch[KSU_AVAR_PIT2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AVAR_PIT2].idNode = 3;
    t->chTable.ch[KSU_AVAR_PIT2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AVAR_PIT2].idCh = 0;
    t->chTable.ch[KSU_AVAR_PIT2].setting.numAdapter = 1;
    t->chTable.ch[KSU_AVAR_PIT2].setting.numCh = 34;
    t->chTable.ch[KSU_AVAR_PIT2].setting.ioCh = 1;

    //АэрПит1
    t->chTable.ch[KSU_AEROD_PIT1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AEROD_PIT1].idNode = 3;
    t->chTable.ch[KSU_AEROD_PIT1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AEROD_PIT1].idCh = 0;
    t->chTable.ch[KSU_AEROD_PIT1].setting.numAdapter = 1;
    t->chTable.ch[KSU_AEROD_PIT1].setting.numCh = 35;
    t->chTable.ch[KSU_AEROD_PIT1].setting.ioCh = 1;

    //АэрПит2
    t->chTable.ch[KSU_AEROD_PIT2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AEROD_PIT2].idNode = 3;
    t->chTable.ch[KSU_AEROD_PIT2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AEROD_PIT2].idCh = 0;
    t->chTable.ch[KSU_AEROD_PIT2].setting.numAdapter = 1;
    t->chTable.ch[KSU_AEROD_PIT2].setting.numCh = 36;
    t->chTable.ch[KSU_AEROD_PIT2].setting.ioCh = 1;

    //Вып шасси осн
    t->chTable.ch[BPS58_VIP_SHASS_OSN].typeNode = E_NODE_PV;
    t->chTable.ch[BPS58_VIP_SHASS_OSN].idNode = 3;
    t->chTable.ch[BPS58_VIP_SHASS_OSN].typeCh = E_CH_RK;
    t->chTable.ch[BPS58_VIP_SHASS_OSN].idCh = 0;
    t->chTable.ch[BPS58_VIP_SHASS_OSN].setting.numAdapter = 1;
    t->chTable.ch[BPS58_VIP_SHASS_OSN].setting.numCh = 37;
    t->chTable.ch[BPS58_VIP_SHASS_OSN].setting.ioCh = 1;

    //Уборка  шасси осн
    t->chTable.ch[BPS58_UBOR_SHASS_OSN].typeNode = E_NODE_PV;
    t->chTable.ch[BPS58_UBOR_SHASS_OSN].idNode = 3;
    t->chTable.ch[BPS58_UBOR_SHASS_OSN].typeCh = E_CH_RK;
    t->chTable.ch[BPS58_UBOR_SHASS_OSN].idCh = 0;
    t->chTable.ch[BPS58_UBOR_SHASS_OSN].setting.numAdapter = 1;
    t->chTable.ch[BPS58_UBOR_SHASS_OSN].setting.numCh = 38;
    t->chTable.ch[BPS58_UBOR_SHASS_OSN].setting.ioCh = 1;

    //Вып створ авар
    t->chTable.ch[BPS58_VIP_ST_AVAR].typeNode = E_NODE_PV;
    t->chTable.ch[BPS58_VIP_ST_AVAR].idNode = 3;
    t->chTable.ch[BPS58_VIP_ST_AVAR].typeCh = E_CH_RK;
    t->chTable.ch[BPS58_VIP_ST_AVAR].idCh = 0;
    t->chTable.ch[BPS58_VIP_ST_AVAR].setting.numAdapter = 1;
    t->chTable.ch[BPS58_VIP_ST_AVAR].setting.numCh = 39;
    t->chTable.ch[BPS58_VIP_ST_AVAR].setting.ioCh = 1;

    //Вып шасси авар
    t->chTable.ch[BPS58_VIP_SHASS_AVAR].typeNode = E_NODE_PV;
    t->chTable.ch[BPS58_VIP_SHASS_AVAR].idNode = 3;
    t->chTable.ch[BPS58_VIP_SHASS_AVAR].typeCh = E_CH_RK;
    t->chTable.ch[BPS58_VIP_SHASS_AVAR].idCh = 0;
    t->chTable.ch[BPS58_VIP_SHASS_AVAR].setting.numAdapter = 1;
    t->chTable.ch[BPS58_VIP_SHASS_AVAR].setting.numCh = 40;
    t->chTable.ch[BPS58_VIP_SHASS_AVAR].setting.ioCh = 1;

    //Отказ ГС1
    t->chTable.ch[KSU_OTKAZ_GS1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OTKAZ_GS1].idNode = 3;
    t->chTable.ch[KSU_OTKAZ_GS1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_OTKAZ_GS1].idCh = 0;
    t->chTable.ch[KSU_OTKAZ_GS1].setting.numAdapter = 1;
    t->chTable.ch[KSU_OTKAZ_GS1].setting.numCh = 41;
    t->chTable.ch[KSU_OTKAZ_GS1].setting.ioCh = 1;

    //Отказ ГС2
    t->chTable.ch[KSU_OTKAZ_GS2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OTKAZ_GS2].idNode = 3;
    t->chTable.ch[KSU_OTKAZ_GS2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_OTKAZ_GS2].idCh = 0;
    t->chTable.ch[KSU_OTKAZ_GS2].setting.numAdapter = 1;
    t->chTable.ch[KSU_OTKAZ_GS2].setting.numCh = 42;
    t->chTable.ch[KSU_OTKAZ_GS2].setting.ioCh = 1;

    //Давл стоян ГС
    t->chTable.ch[KSU_DAV_ST_GS].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_DAV_ST_GS].idNode = 3;
    t->chTable.ch[KSU_DAV_ST_GS].typeCh = E_CH_RK;
    t->chTable.ch[KSU_DAV_ST_GS].idCh = 0;
    t->chTable.ch[KSU_DAV_ST_GS].setting.numAdapter = 1;
    t->chTable.ch[KSU_DAV_ST_GS].setting.numCh = 44;
    t->chTable.ch[KSU_DAV_ST_GS].setting.ioCh = 1;

    //! КУТР БКС
    arTable.setCh(&(t->chTable.ch[AR_IN_BKS4_KUTR]));
    arTable.setAddr(E_NODE_PV, 1);
    arTable.setChId(E_CH_AR, 3, 0);
    arTable << 0100 << 0101 << 0102 << 0103 << 0104 << 0105 << 0106 << 0110 <<0111 <<0112 <<0113
            << 0130 << 0131 << 0132 << 0133 ;
    arTable.setProp(LayerArinc::KBs_12_5, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! БИНС 1
    arTable.setCh(&(t->chTable.ch[AR_OUT_BINS1]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 4, 1);
    arTable << 0270 << 0310 << 0311 << 0312 << 0313 << 0335 << 0314 << 0317
            << 0320 << 0324 << 0325 << 0334 << 0226 << 0366 << 0367 << 0365
            << 0360 << 0327 << 0330 << 0326 << 0331 << 0333 << 0332 << 0364
            << 0361 << 0315 << 0316 << 0321 << 0322;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! БИНС 2
    arTable.setCh(&(t->chTable.ch[AR_OUT_BINS2]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 5, 1);
    arTable << 0270 << 0310 << 0311 << 0312 << 0313 << 0335 << 0314 << 0317
            << 0320 << 0324 << 0325 << 0334 << 0226 << 0366 << 0367 << 0365
            << 0360 << 0327 << 0330 << 0326 << 0331 << 0333 << 0332 << 0364
            << 0361 << 0315 << 0316 << 0321 << 0322;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! БИНС 3
    arTable.setCh(&(t->chTable.ch[AR_OUT_BINS3]));
    arTable.setAddr(E_NODE_CV, 1);
    arTable.setChId(E_CH_AR, 6, 1);
    arTable << 0270 << 0310 << 0311 << 0312 << 0313 << 0335 << 0314 << 0317
            << 0320 << 0324 << 0325 << 0334 << 0226 << 0366 << 0367 << 0365
            << 0360 << 0327 << 0330 << 0326 << 0331 << 0333 << 0332 << 0364
            << 0361 << 0315 << 0316 << 0321 << 0322;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);



//    //ЕВ1_БКС1
//    t->chTable.ch[SVO_EV1_BKS1].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV1_BKS1].idNode = 1;
//    t->chTable.ch[SVO_EV1_BKS1].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV1_BKS1].idCh = 0;
//    t->chTable.ch[SVO_EV1_BKS1].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV1_BKS1].setting.numCh = 1;
//    t->chTable.ch[SVO_EV1_BKS1].setting.ioCh = 1;
//    //ЕВ2_БКС4
//    t->chTable.ch[SVO_EV2_BKS4].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV2_BKS4].idNode = 1;
//    t->chTable.ch[SVO_EV2_BKS4].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV2_BKS4].idCh = 0;
//    t->chTable.ch[SVO_EV2_BKS4].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV2_BKS4].setting.numCh = 2;
//    t->chTable.ch[SVO_EV2_BKS4].setting.ioCh = 1;
//    //ЕВ3_БКС2
//    t->chTable.ch[SVO_EV3_BKS2].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV3_BKS2].idNode = 1;
//    t->chTable.ch[SVO_EV3_BKS2].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV3_BKS2].idCh = 0;
//    t->chTable.ch[SVO_EV3_BKS2].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV3_BKS2].setting.numCh = 3;
//    t->chTable.ch[SVO_EV3_BKS2].setting.ioCh = 1;
//    //ЕВ3_БКС4
//    t->chTable.ch[SVO_EV3_BKS4].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV3_BKS4].idNode = 1;
//    t->chTable.ch[SVO_EV3_BKS4].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV3_BKS4].idCh = 0;
//    t->chTable.ch[SVO_EV3_BKS4].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV3_BKS4].setting.numCh = 4;
//    t->chTable.ch[SVO_EV3_BKS4].setting.ioCh = 1;
//    //ЕВ4_БКС2
//    t->chTable.ch[SVO_EV4_BKS2].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV4_BKS2].idNode = 1;
//    t->chTable.ch[SVO_EV4_BKS2].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV4_BKS2].idCh = 0;
//    t->chTable.ch[SVO_EV4_BKS2].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV4_BKS2].setting.numCh = 5;
//    t->chTable.ch[SVO_EV4_BKS2].setting.ioCh = 1;
//
//    //ЕВ4_БКС4
//    t->chTable.ch[SVO_EV4_BKS4].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV4_BKS4].idNode = 1;
//    t->chTable.ch[SVO_EV4_BKS4].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV4_BKS4].idCh = 0;
//    t->chTable.ch[SVO_EV4_BKS4].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV4_BKS4].setting.numCh = 6;
//    t->chTable.ch[SVO_EV4_BKS4].setting.ioCh = 1;
//
//    //ЕВ5_БКС1
//    t->chTable.ch[SVO_EV5_BKS1].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS1].idNode = 1;
//    t->chTable.ch[SVO_EV5_BKS1].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS1].idCh = 0;
//    t->chTable.ch[SVO_EV5_BKS1].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV5_BKS1].setting.numCh = 7;
//    t->chTable.ch[SVO_EV5_BKS1].setting.ioCh = 1;
//
//    //ЕВ5_БКС2
//    t->chTable.ch[SVO_EV5_BKS2].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS2].idNode = 1;
//    t->chTable.ch[SVO_EV5_BKS2].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS2].idCh = 0;
//    t->chTable.ch[SVO_EV5_BKS2].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV5_BKS2].setting.numCh = 8;
//    t->chTable.ch[SVO_EV5_BKS2].setting.ioCh = 1;
//
//    //ЕВ5_БКС3
//    t->chTable.ch[SVO_EV5_BKS3].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS3].idNode = 1;
//    t->chTable.ch[SVO_EV5_BKS3].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS3].idCh = 0;
//    t->chTable.ch[SVO_EV5_BKS3].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV5_BKS3].setting.numCh = 9;
//    t->chTable.ch[SVO_EV5_BKS3].setting.ioCh = 1;
//
//    //ЕВ5_БКС4
//    t->chTable.ch[SVO_EV5_BKS4].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS4].idNode = 1;
//    t->chTable.ch[SVO_EV5_BKS4].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS4].idCh = 0;
//    t->chTable.ch[SVO_EV5_BKS4].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV5_BKS4].setting.numCh = 10;
//    t->chTable.ch[SVO_EV5_BKS4].setting.ioCh = 1;
//
//    //ЕВ6_БКС1
//    t->chTable.ch[SVO_EV6_BKS1].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS1].idNode = 1;
//    t->chTable.ch[SVO_EV6_BKS1].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS1].idCh = 0;
//    t->chTable.ch[SVO_EV6_BKS1].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV6_BKS1].setting.numCh = 11;
//    t->chTable.ch[SVO_EV6_BKS1].setting.ioCh = 1;
//
//    //ЕВ6_БКС2
//    t->chTable.ch[SVO_EV6_BKS2].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS2].idNode = 1;
//    t->chTable.ch[SVO_EV6_BKS2].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS2].idCh = 0;
//    t->chTable.ch[SVO_EV6_BKS2].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV6_BKS2].setting.numCh = 12;
//    t->chTable.ch[SVO_EV6_BKS2].setting.ioCh = 1;
//
//    //ЕВ6_БКС3
//    t->chTable.ch[SVO_EV6_BKS3].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS3].idNode = 1;
//    t->chTable.ch[SVO_EV6_BKS3].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS3].idCh = 0;
//    t->chTable.ch[SVO_EV6_BKS3].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV6_BKS3].setting.numCh = 13;
//    t->chTable.ch[SVO_EV6_BKS3].setting.ioCh = 1;
//
//    //ЕВ6_БКС4
//    t->chTable.ch[SVO_EV6_BKS4].typeNode = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS4].idNode = 1;
//    t->chTable.ch[SVO_EV6_BKS4].typeCh = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS4].idCh = 0;
//    t->chTable.ch[SVO_EV6_BKS4].setting.numAdapter = 1;
//    t->chTable.ch[SVO_EV6_BKS4].setting.numCh = 14;
//    t->chTable.ch[SVO_EV6_BKS4].setting.ioCh = 1;

    //Давление в первой гидросистеме выше 100 кгс/см2
    t->chTable.ch[HPS_ISPR_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ISPR_1GS].idNode = 1;
    t->chTable.ch[HPS_ISPR_1GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ISPR_1GS].idCh = 0;
    t->chTable.ch[HPS_ISPR_1GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ISPR_1GS].setting.numCh = 48 + 13;
    t->chTable.ch[HPS_ISPR_1GS].setting.ioCh = 1;
    //Давление в первой гидросистеме выше 100 кгс/см2
    t->chTable.ch[HPS_ISPR_1GS_BKS2].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ISPR_1GS_BKS2].idNode = 1;
    t->chTable.ch[HPS_ISPR_1GS_BKS2].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ISPR_1GS_BKS2].idCh = 0;
    t->chTable.ch[HPS_ISPR_1GS_BKS2].setting.numAdapter = 1;
    t->chTable.ch[HPS_ISPR_1GS_BKS2].setting.numCh = 48 + 13;
    t->chTable.ch[HPS_ISPR_1GS_BKS2].setting.ioCh = 1;
    //Давление во второй гидросистеме выше 100 кгс/см2
    t->chTable.ch[HPS_ISPR_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ISPR_2GS].idNode = 1;
    t->chTable.ch[HPS_ISPR_2GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ISPR_2GS].idCh = 0;
    t->chTable.ch[HPS_ISPR_2GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ISPR_2GS].setting.numCh = 48 + 14;
    t->chTable.ch[HPS_ISPR_2GS].setting.ioCh = 1;
    //Давление во второй гидросистеме выше 100 кгс/см2
    t->chTable.ch[HPS_ISPR_2GS_BKS2].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ISPR_2GS_BKS2].idNode = 1;
    t->chTable.ch[HPS_ISPR_2GS_BKS2].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ISPR_2GS_BKS2].idCh = 0;
    t->chTable.ch[HPS_ISPR_2GS_BKS2].setting.numAdapter = 1;
    t->chTable.ch[HPS_ISPR_2GS_BKS2].setting.numCh = 48 + 14;
    t->chTable.ch[HPS_ISPR_2GS_BKS2].setting.ioCh = 1;

    //Засорение фильтра слива 1ГС
    t->chTable.ch[HPS_ZASOR_FS_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ZASOR_FS_1GS].idNode = 1;
    t->chTable.ch[HPS_ZASOR_FS_1GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ZASOR_FS_1GS].idCh = 0;
    t->chTable.ch[HPS_ZASOR_FS_1GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ZASOR_FS_1GS].setting.numCh = 48 + 15;
    t->chTable.ch[HPS_ZASOR_FS_1GS].setting.ioCh = 1;
    //Засорение фильтра слива 2ГС
    t->chTable.ch[HPS_ZASOR_FS_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ZASOR_FS_2GS].idNode = 1;
    t->chTable.ch[HPS_ZASOR_FS_2GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ZASOR_FS_2GS].idCh = 0;
    t->chTable.ch[HPS_ZASOR_FS_2GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ZASOR_FS_2GS].setting.numCh = 48 + 16;
    t->chTable.ch[HPS_ZASOR_FS_2GS].setting.ioCh = 1;
    //Засорение фильтра слива НП 1ГС
    t->chTable.ch[HPS_ZASOR_FNP_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ZASOR_FNP_1GS].idNode = 1;
    t->chTable.ch[HPS_ZASOR_FNP_1GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ZASOR_FNP_1GS].idCh = 0;
    t->chTable.ch[HPS_ZASOR_FNP_1GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ZASOR_FNP_1GS].setting.numCh = 48 + 17;
    t->chTable.ch[HPS_ZASOR_FNP_1GS].setting.ioCh = 1;
    //Засорение фильтра слива НП 2ГС
    t->chTable.ch[HPS_ZASOR_FNP_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ZASOR_FNP_2GS].idNode = 1;
    t->chTable.ch[HPS_ZASOR_FNP_2GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ZASOR_FNP_2GS].idCh = 0;
    t->chTable.ch[HPS_ZASOR_FNP_2GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ZASOR_FNP_2GS].setting.numCh = 48 + 18;
    t->chTable.ch[HPS_ZASOR_FNP_2GS].setting.ioCh = 1;
    //Засорение фильтра нагнетания 1ГС
    t->chTable.ch[HPS_ZASOR_FN_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ZASOR_FN_1GS].idNode = 1;
    t->chTable.ch[HPS_ZASOR_FN_1GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ZASOR_FN_1GS].idCh = 0;
    t->chTable.ch[HPS_ZASOR_FN_1GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ZASOR_FN_1GS].setting.numCh = 48 + 19;
    t->chTable.ch[HPS_ZASOR_FN_1GS].setting.ioCh = 1;
    //Засорение фильтра нагнетания 2ГС
    t->chTable.ch[HPS_ZASOR_FN_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_ZASOR_FN_2GS].idNode = 1;
    t->chTable.ch[HPS_ZASOR_FN_2GS].typeCh = E_CH_RK;
    t->chTable.ch[HPS_ZASOR_FN_2GS].idCh = 0;
    t->chTable.ch[HPS_ZASOR_FN_2GS].setting.numAdapter = 1;
    t->chTable.ch[HPS_ZASOR_FN_2GS].setting.numCh = 48 + 20;
    t->chTable.ch[HPS_ZASOR_FN_2GS].setting.ioCh = 1;
    //Сигнал расцепления левого привода-генератора
    t->chTable.ch[SES_RASTEP_LGSN].typeNode = E_NODE_PV;
    t->chTable.ch[SES_RASTEP_LGSN].idNode = 1;
    t->chTable.ch[SES_RASTEP_LGSN].typeCh = E_CH_RK;
    t->chTable.ch[SES_RASTEP_LGSN].idCh = 0;
    t->chTable.ch[SES_RASTEP_LGSN].setting.numAdapter = 1;
    t->chTable.ch[SES_RASTEP_LGSN].setting.numCh = 48 + 21;
    t->chTable.ch[SES_RASTEP_LGSN].setting.ioCh = 1;
    //Отказ левого канала переменного тока
    t->chTable.ch[SES_OTK_LKANPERI_1].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_LKANPERI_1].idNode = 1;
    t->chTable.ch[SES_OTK_LKANPERI_1].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_LKANPERI_1].idCh = 0;
    t->chTable.ch[SES_OTK_LKANPERI_1].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_LKANPERI_1].setting.numCh = 48 + 22;
    t->chTable.ch[SES_OTK_LKANPERI_1].setting.ioCh = 1;

    //Отказ левого канала переменного тока
    t->chTable.ch[SES_OTK_LKANPERI_2].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_LKANPERI_2].idNode = 1;
    t->chTable.ch[SES_OTK_LKANPERI_2].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_LKANPERI_2].idCh = 0;
    t->chTable.ch[SES_OTK_LKANPERI_2].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_LKANPERI_2].setting.numCh = 48 + 23;
    t->chTable.ch[SES_OTK_LKANPERI_2].setting.ioCh = 1;

    //Отказ правого канала переменного тока
    t->chTable.ch[SES_OTK_RKANPERI_1].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_RKANPERI_1].idNode = 1;
    t->chTable.ch[SES_OTK_RKANPERI_1].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_RKANPERI_1].idCh = 0;
    t->chTable.ch[SES_OTK_RKANPERI_1].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_RKANPERI_1].setting.numCh = 48 + 24;
    t->chTable.ch[SES_OTK_RKANPERI_1].setting.ioCh = 1;

    //Отказ правого канала переменного тока
    t->chTable.ch[SES_OTK_RKANPERI_2].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_RKANPERI_2].idNode = 1;
    t->chTable.ch[SES_OTK_RKANPERI_2].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_RKANPERI_2].idCh = 0;
    t->chTable.ch[SES_OTK_RKANPERI_2].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_RKANPERI_2].setting.numCh = 48 + 25;
    t->chTable.ch[SES_OTK_RKANPERI_2].setting.ioCh = 1;

    //Отказ левого привода-генератора-фидера
    t->chTable.ch[SES_OTKLPRIVGENFID].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTKLPRIVGENFID].idNode = 1;
    t->chTable.ch[SES_OTKLPRIVGENFID].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTKLPRIVGENFID].idCh = 0;
    t->chTable.ch[SES_OTKLPRIVGENFID].setting.numAdapter = 1;
    t->chTable.ch[SES_OTKLPRIVGENFID].setting.numCh = 48 + 26;
    t->chTable.ch[SES_OTKLPRIVGENFID].setting.ioCh = 1;

    //Отказ правого генератора-фидера
    t->chTable.ch[SES_OTKRGENFID].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTKRGENFID].idNode = 1;
    t->chTable.ch[SES_OTKRGENFID].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTKRGENFID].idCh = 0;
    t->chTable.ch[SES_OTKRGENFID].setting.numAdapter = 1;
    t->chTable.ch[SES_OTKRGENFID].setting.numCh = 48 + 27;
    t->chTable.ch[SES_OTKRGENFID].setting.ioCh = 1;

    //Отказ БРЗУ левого канала
    t->chTable.ch[SES_OTK_BRZULEFT].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_BRZULEFT].idNode = 1;
    t->chTable.ch[SES_OTK_BRZULEFT].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_BRZULEFT].idCh = 0;
    t->chTable.ch[SES_OTK_BRZULEFT].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_BRZULEFT].setting.numCh = 48 + 28;
    t->chTable.ch[SES_OTK_BRZULEFT].setting.ioCh = 1;

    //Отказ БРЗУ правого канала
    t->chTable.ch[SES_OTK_BRZURIGHT].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_BRZURIGHT].idNode = 1;
    t->chTable.ch[SES_OTK_BRZURIGHT].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_BRZURIGHT].idCh = 0;
    t->chTable.ch[SES_OTK_BRZURIGHT].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_BRZURIGHT].setting.numCh = 48 + 29;
    t->chTable.ch[SES_OTK_BRZURIGHT].setting.ioCh = 1;

    //Отказ одного ВУ
    t->chTable.ch[SES_OTK_1_VU].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_1_VU].idNode = 1;
    t->chTable.ch[SES_OTK_1_VU].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_1_VU].idCh = 0;
    t->chTable.ch[SES_OTK_1_VU].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_1_VU].setting.numCh = 48 + 30;
    t->chTable.ch[SES_OTK_1_VU].setting.ioCh = 1;

    //Отказ двух ВУ
    t->chTable.ch[SES_OTK_2_VU].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_2_VU].idNode = 1;
    t->chTable.ch[SES_OTK_2_VU].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_2_VU].idCh = 0;
    t->chTable.ch[SES_OTK_2_VU].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_2_VU].setting.numCh = 48 + 31;
    t->chTable.ch[SES_OTK_2_VU].setting.ioCh = 1;

    //Отказ трех ВУ
    t->chTable.ch[SES_OTK_3_VU_1].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_3_VU_1].idNode = 1;
    t->chTable.ch[SES_OTK_3_VU_1].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_3_VU_1].idCh = 0;
    t->chTable.ch[SES_OTK_3_VU_1].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_3_VU_1].setting.numCh = 48 + 32;
    t->chTable.ch[SES_OTK_3_VU_1].setting.ioCh = 1;

    //Отказ трех ВУ
    t->chTable.ch[SES_OTK_3_VU_2].typeNode = E_NODE_PV;
    t->chTable.ch[SES_OTK_3_VU_2].idNode = 1;
    t->chTable.ch[SES_OTK_3_VU_2].typeCh = E_CH_RK;
    t->chTable.ch[SES_OTK_3_VU_2].idCh = 0;
    t->chTable.ch[SES_OTK_3_VU_2].setting.numAdapter = 1;
    t->chTable.ch[SES_OTK_3_VU_2].setting.numCh = 48 + 33;
    t->chTable.ch[SES_OTK_3_VU_2].setting.ioCh = 1;

    //! БКС1
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 1;
    mil->numWord = 21;

    t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 2;
    mil->numWord = 9;

    t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 3;
    mil->numWord = 3;

    t->chTable.ch[MIL_IN_BKS1_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS1_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 1;
    mil->subAddr = 1;
    mil->numWord = 4;

    t->chTable.ch[MIL_IN_BKS1_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BKS1_NVG_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS1_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 1;
    mil->subAddr = 2;
    mil->numWord = 1;

    //! БКС2
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_OSO_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 1;
    mil->numWord = 20;

    t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_OSO_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 2;
    mil->numWord = 9;

    t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_OSO_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 3;
    mil->numWord = 2;

    t->chTable.ch[MIL_IN_BKS2_OSO_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS2_OSO_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 12;
    mil->subAddr = 1;
    mil->numWord = 4;

    t->chTable.ch[MIL_IN_BKS2_OSO_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BKS2_OSO_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS2_OSO_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 12;
    mil->subAddr = 2;
    mil->numWord = 1;

    //! БКС3
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 2;
    mil->subAddr = 1;
    mil->numWord = 20;

    t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 2;
    mil->subAddr = 2;
    mil->numWord = 9;

    t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 2;
    mil->subAddr = 3;
    mil->numWord = 6;

    ///!
//    t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].typeNode = E_NODE_CV;
//    t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].idNode = 1;
//    t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].typeCh = E_CH_MIL;
//    t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].idCh = 0;
//    t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].setting.numAdapter = 0;
//    t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].setting.numCh = LayerMIL::CH_MIL_NVG;
//    t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].setting.ioCh = 1;
//    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_NVG_SA4].desData);
//    mil->typeTrans = LayerMIL::OUK;
//    mil->addr = 2;
//    mil->subAddr = 4;
//    mil->numWord = 2;

    t->chTable.ch[MIL_IN_BKS3_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS3_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 2;
    mil->subAddr = 1;
    mil->numWord = 4;

    t->chTable.ch[MIL_IN_BKS3_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG;
    t->chTable.ch[MIL_IN_BKS3_NVG_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS3_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 2;
    mil->subAddr = 2;
    mil->numWord = 1;

    //! БКС4
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_OSO_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 1;
    mil->numWord = 20;

    t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_OSO_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 2;
    mil->numWord = 14;

    t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_OSO_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 3;
    mil->numWord = 9;

    t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].idNode = 1;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_OSO_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 4;
    mil->numWord = 2;

    t->chTable.ch[MIL_IN_BKS4_OSO_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA1].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS4_OSO_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 13;
    mil->subAddr = 1;
    mil->numWord = 4;

    t->chTable.ch[MIL_IN_BKS4_OSO_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA2].setting.numCh = LayerMIL::CH_MIL_OSO;
    t->chTable.ch[MIL_IN_BKS4_OSO_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS4_OSO_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 13;
    mil->subAddr = 2;
    mil->numWord = 1;

    //!  P-104
    t->chTable.ch[SVS_P_104_1].typeNode = E_NODE_PV;
    t->chTable.ch[SVS_P_104_1].idNode = 1;
    t->chTable.ch[SVS_P_104_1].typeCh = E_IR;
    t->chTable.ch[SVS_P_104_1].idCh = 0;
    t->chTable.ch[SVS_P_104_1].setting.numAdapter = 0;
    t->chTable.ch[SVS_P_104_1].setting.numCh = 15;
    t->chTable.ch[SVS_P_104_1].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[SVS_P_104_1].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = 2574;
    //!  P-104
    t->chTable.ch[SVS_P_104_2].typeNode = E_NODE_PV;
    t->chTable.ch[SVS_P_104_2].idNode = 1;
    t->chTable.ch[SVS_P_104_2].typeCh = E_IR;
    t->chTable.ch[SVS_P_104_2].idCh = 0;
    t->chTable.ch[SVS_P_104_2].setting.numAdapter = 0;
    t->chTable.ch[SVS_P_104_2].setting.numCh = 16;
    t->chTable.ch[SVS_P_104_2].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[SVS_P_104_2].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = 2574;
    //!  HPS_TRJ_1GS
    t->chTable.ch[HPS_TRJ_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_TRJ_1GS].idNode = 1;
    t->chTable.ch[HPS_TRJ_1GS].typeCh = E_IR;
    t->chTable.ch[HPS_TRJ_1GS].idCh = 0;
    t->chTable.ch[HPS_TRJ_1GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_TRJ_1GS].setting.numCh = 4;
    t->chTable.ch[HPS_TRJ_1GS].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_TRJ_1GS].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;
    //HPS_TRJ_1GS_BKS2,
    t->chTable.ch[HPS_TRJ_1GS_BKS2].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_TRJ_1GS_BKS2].idNode = 1;
    t->chTable.ch[HPS_TRJ_1GS_BKS2].typeCh = E_IR;
    t->chTable.ch[HPS_TRJ_1GS_BKS2].idCh = 0;
    t->chTable.ch[HPS_TRJ_1GS_BKS2].setting.numAdapter = 0;
    t->chTable.ch[HPS_TRJ_1GS_BKS2].setting.numCh = 5;
    t->chTable.ch[HPS_TRJ_1GS_BKS2].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_TRJ_1GS_BKS2].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;
    //HPS_TRJ_2GS
    t->chTable.ch[HPS_TRJ_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_TRJ_2GS].idNode = 1;
    t->chTable.ch[HPS_TRJ_2GS].typeCh = E_IR;
    t->chTable.ch[HPS_TRJ_2GS].idCh = 0;
    t->chTable.ch[HPS_TRJ_2GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_TRJ_2GS].setting.numCh = 6;
    t->chTable.ch[HPS_TRJ_2GS].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_TRJ_2GS].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;
    //HPS_TRJ_2GS_BKS2
    t->chTable.ch[HPS_TRJ_2GS_BKS2].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_TRJ_2GS_BKS2].idNode = 1;
    t->chTable.ch[HPS_TRJ_2GS_BKS2].typeCh = E_IR;
    t->chTable.ch[HPS_TRJ_2GS_BKS2].idCh = 0;
    t->chTable.ch[HPS_TRJ_2GS_BKS2].setting.numAdapter = 0;
    t->chTable.ch[HPS_TRJ_2GS_BKS2].setting.numCh = 7;
    t->chTable.ch[HPS_TRJ_2GS_BKS2].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_TRJ_2GS_BKS2].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    //HPS_PERSHT_1GS
    t->chTable.ch[HPS_PERSHT_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_PERSHT_1GS].idNode = 1;
    t->chTable.ch[HPS_PERSHT_1GS].typeCh = E_IR;
    t->chTable.ch[HPS_PERSHT_1GS].idCh = 0;
    t->chTable.ch[HPS_PERSHT_1GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_PERSHT_1GS].setting.numCh = 10;
    t->chTable.ch[HPS_PERSHT_1GS].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_PERSHT_1GS].desData);
    ir->max = 2000;
    ir->min = 0;
    ir->value = ir->min;

    //HPS_PERSHT_1GS_BKS2
    t->chTable.ch[HPS_PERSHT_1GS_BKS2].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_PERSHT_1GS_BKS2].idNode = 1;
    t->chTable.ch[HPS_PERSHT_1GS_BKS2].typeCh = E_IR;
    t->chTable.ch[HPS_PERSHT_1GS_BKS2].idCh = 0;
    t->chTable.ch[HPS_PERSHT_1GS_BKS2].setting.numAdapter = 0;
    t->chTable.ch[HPS_PERSHT_1GS_BKS2].setting.numCh = 11;
    t->chTable.ch[HPS_PERSHT_1GS_BKS2].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_PERSHT_1GS_BKS2].desData);
    ir->max = 2000;
    ir->min = 0;
    ir->value = ir->min;
    //HPS_PERSHT_2GS
    t->chTable.ch[HPS_PERSHT_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_PERSHT_2GS].idNode = 1;
    t->chTable.ch[HPS_PERSHT_2GS].typeCh = E_IR;
    t->chTable.ch[HPS_PERSHT_2GS].idCh = 0;
    t->chTable.ch[HPS_PERSHT_2GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_PERSHT_2GS].setting.numCh = 22;
    t->chTable.ch[HPS_PERSHT_2GS].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_PERSHT_2GS].desData);
    ir->max = 2000;
    ir->min = 0;
    ir->value = ir->min;
    //HPS_PERSHT_2GS_BKS2
    t->chTable.ch[HPS_PERSHT_2GS_BKS2].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_PERSHT_2GS_BKS2].idNode = 1;
    t->chTable.ch[HPS_PERSHT_2GS_BKS2].typeCh = E_IR;
    t->chTable.ch[HPS_PERSHT_2GS_BKS2].idCh = 0;
    t->chTable.ch[HPS_PERSHT_2GS_BKS2].setting.numAdapter = 0;
    t->chTable.ch[HPS_PERSHT_2GS_BKS2].setting.numCh = 23;
    t->chTable.ch[HPS_PERSHT_2GS_BKS2].setting.ioCh = 1;
    ir = (TDesIR*) (t->chTable.ch[HPS_PERSHT_2GS_BKS2].desData);
    ir->max = 2000;
    ir->min = 0;
    ir->value = ir->min;

    //HPS_P_1GS
    t->chTable.ch[HPS_P_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_1GS].idNode = 1;
    t->chTable.ch[HPS_P_1GS].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_1GS].idCh = 0;
    t->chTable.ch[HPS_P_1GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_1GS].setting.numCh = 0;
    t->chTable.ch[HPS_P_1GS].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_1GS].desData);
    dac->volt = 0;

    //HPS_P_2GS
    t->chTable.ch[HPS_P_1GS_BKS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_1GS_BKS].idNode = 1;
    t->chTable.ch[HPS_P_1GS_BKS].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_1GS_BKS].idCh = 0;
    t->chTable.ch[HPS_P_1GS_BKS].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_1GS_BKS].setting.numCh = 1;
    t->chTable.ch[HPS_P_1GS_BKS].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_1GS_BKS].desData);
    dac->volt = 0;

    //HPS_P_1GS_BKS
    t->chTable.ch[HPS_P_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_2GS].idNode = 1;
    t->chTable.ch[HPS_P_2GS].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_2GS].idCh = 0;
    t->chTable.ch[HPS_P_2GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_2GS].setting.numCh = 2;
    t->chTable.ch[HPS_P_2GS].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_2GS].desData);
    dac->volt = 0;

    //HPS_P_2GS_BKS
    t->chTable.ch[HPS_P_2GS_BKS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_2GS_BKS].idNode = 1;
    t->chTable.ch[HPS_P_2GS_BKS].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_2GS_BKS].idCh = 0;
    t->chTable.ch[HPS_P_2GS_BKS].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_2GS_BKS].setting.numCh = 3;
    t->chTable.ch[HPS_P_2GS_BKS].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_2GS_BKS].desData);
    dac->volt = 0;

    //HPS_P_GA_1GS
    t->chTable.ch[HPS_P_GA_1GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_GA_1GS].idNode = 1;
    t->chTable.ch[HPS_P_GA_1GS].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_GA_1GS].idCh = 0;
    t->chTable.ch[HPS_P_GA_1GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_GA_1GS].setting.numCh = 4;
    t->chTable.ch[HPS_P_GA_1GS].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_GA_1GS].desData);
    dac->volt = 0;

    //HPS_P_GA_2GS
    t->chTable.ch[HPS_P_GA_2GS].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_GA_2GS].idNode = 1;
    t->chTable.ch[HPS_P_GA_2GS].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_GA_2GS].idCh = 0;
    t->chTable.ch[HPS_P_GA_2GS].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_GA_2GS].setting.numCh = 5;
    t->chTable.ch[HPS_P_GA_2GS].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_GA_2GS].desData);
    dac->volt = 0;

    //HPS_P_PNEV
    t->chTable.ch[HPS_P_PNEV].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_PNEV].idNode = 1;
    t->chTable.ch[HPS_P_PNEV].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_PNEV].idCh = 0;
    t->chTable.ch[HPS_P_PNEV].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_PNEV].setting.numCh = 6;
    t->chTable.ch[HPS_P_PNEV].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_PNEV].desData);
    dac->volt = 0;

    //HPS_P_PNEV_BKS2
    t->chTable.ch[HPS_P_PNEV_BKS2].typeNode = E_NODE_PV;
    t->chTable.ch[HPS_P_PNEV_BKS2].idNode = 1;
    t->chTable.ch[HPS_P_PNEV_BKS2].typeCh = E_CH_DAC;
    t->chTable.ch[HPS_P_PNEV_BKS2].idCh = 0;
    t->chTable.ch[HPS_P_PNEV_BKS2].setting.numAdapter = 0;
    t->chTable.ch[HPS_P_PNEV_BKS2].setting.numCh = 7;
    t->chTable.ch[HPS_P_PNEV_BKS2].setting.ioCh = 1;
    dac = (TDesDAC*) (t->chTable.ch[HPS_P_PNEV_BKS2].desData);
    dac->volt = 0;

    //SES_I_VU1
    t->chTable.ch[SES_I_VU1].typeNode = E_NODE_PV;
    t->chTable.ch[SES_I_VU1].idNode = 1;
    t->chTable.ch[SES_I_VU1].typeCh = E_ITP;
    t->chTable.ch[SES_I_VU1].idCh = 0;
    t->chTable.ch[SES_I_VU1].setting.numAdapter = 0;
    t->chTable.ch[SES_I_VU1].setting.numCh = 6;
    t->chTable.ch[SES_I_VU1].setting.ioCh = 1;
    itp = (TDesITP*) (t->chTable.ch[SES_I_VU1].desData);
    itp->value = 100;
    itp->type = 50;
    itp->max = 150;

    //SES_I_VU2
    t->chTable.ch[SES_I_VU2].typeNode = E_NODE_PV;
    t->chTable.ch[SES_I_VU2].idNode = 1;
    t->chTable.ch[SES_I_VU2].typeCh = E_ITP;
    t->chTable.ch[SES_I_VU2].idCh = 0;
    t->chTable.ch[SES_I_VU2].setting.numAdapter = 0;
    t->chTable.ch[SES_I_VU2].setting.numCh = 7;
    t->chTable.ch[SES_I_VU2].setting.ioCh = 1;
    itp = (TDesITP*) (t->chTable.ch[SES_I_VU2].desData);
    itp->value = 100;
    itp->type = 50;
    itp->max = 150;

    //SES_I_VU3
    t->chTable.ch[SES_I_VU3].typeNode = E_NODE_PV;
    t->chTable.ch[SES_I_VU3].idNode = 1;
    t->chTable.ch[SES_I_VU3].typeCh = E_ITP;
    t->chTable.ch[SES_I_VU3].idCh = 0;
    t->chTable.ch[SES_I_VU3].setting.numAdapter = 0;
    t->chTable.ch[SES_I_VU3].setting.numCh = 8;
    t->chTable.ch[SES_I_VU3].setting.ioCh = 1;
    itp = (TDesITP*) (t->chTable.ch[SES_I_VU3].desData);
    itp->value = 100;
    itp->type = 50;
    itp->max = 150;

    //GEN A L
    t->chTable.ch[SES_FAZA_A_L].typeNode = E_NODE_PV;
    t->chTable.ch[SES_FAZA_A_L].idNode = 1;
    t->chTable.ch[SES_FAZA_A_L].typeCh = E_GEN_U;
    t->chTable.ch[SES_FAZA_A_L].idCh = 0;
    t->chTable.ch[SES_FAZA_A_L].setting.numAdapter = 0;
    t->chTable.ch[SES_FAZA_A_L].setting.numCh = 0;
    t->chTable.ch[SES_FAZA_A_L].setting.ioCh = 1;
    gen_u = (TDesGen_U*) (t->chTable.ch[SES_FAZA_A_L].desData);
    gen_u->u = 0;
    gen_u->ff = 0;
    gen_u->f = 0;
    gen_u->phase = 7;


    //GEN A R
    t->chTable.ch[SES_FAZA_A_R].typeNode = E_NODE_PV;
    t->chTable.ch[SES_FAZA_A_R].idNode = 1;
    t->chTable.ch[SES_FAZA_A_R].typeCh = E_GEN_U;
    t->chTable.ch[SES_FAZA_A_R].idCh = 0;
    t->chTable.ch[SES_FAZA_A_R].setting.numAdapter = 0;
    t->chTable.ch[SES_FAZA_A_R].setting.numCh = 1;
    t->chTable.ch[SES_FAZA_A_R].setting.ioCh = 1;
    gen_u = (TDesGen_U*) (t->chTable.ch[SES_FAZA_A_R].desData);
    gen_u->u = 0;
    gen_u->ff = 0;
    gen_u->f = 0;
    gen_u->phase = 7;



    //! КСУ(в СБИ)
    arTable.setCh(&(t->chTable.ch[AR_OUT_KSU_L_UCOKS1]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 16 + 6, 1);
    arTable << 0203 << 0204 << 0206 << 0210 << 0205 << 0212 << 0221
            << 0321 << 0240 << 061  << 0302 << 0113 << 0300 << 073
            << 0245 << 0301 << 0364 << 0363 << 0141 << 0140 << 0173
            << 0174 << 0175 << 0176 << 020  << 021  << 0250 << 01
            << 02   << 03   << 04   << 05   << 06   << 011  << 012
            << 013  << 014  << 015  << 016  << 017  << 022  << 023
            << 024  << 025  << 026  << 027  << 030  << 032  ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_KSU_L_UCOKS2]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 16 + 7, 1);
    arTable << 0203 << 0204 << 0206 << 0210 << 0205 << 0212 << 0221
            << 0321 << 0240 << 061  << 0302 << 0113 << 0300 << 073
            << 0245 << 0301 << 0364 << 0363 << 0141 << 0140 << 0173
            << 0174 << 0175 << 0176 << 020  << 021  << 0250 << 01
            << 02   << 03   << 04   << 05   << 06   << 011  << 012
            << 013  << 014  << 015  << 016  << 017  << 022  << 023
            << 024  << 025  << 026  << 027  << 030  << 032  ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_KSU_R_UCOKS1]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 16 + 8, 1);
    arTable << 0203 << 0204 << 0206 << 0210 << 0205 << 0212 << 0221
            << 0321 << 0240 << 061  << 0302 << 0113 << 0300 << 073
            << 0245 << 0301 << 0364 << 0363 << 0141 << 0140 << 0173
            << 0174 << 0175 << 0176 << 020  << 021  << 0250 << 01
            << 02   << 03   << 04   << 05   << 06   << 011  << 012
            << 013  << 014  << 015  << 016  << 017  << 022  << 023
            << 024  << 025  << 026  << 027  << 030  << 032  ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_KSU_R_UCOKS2]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 16 + 9, 1);
    arTable << 0203 << 0204 << 0206 << 0210 << 0205 << 0212 << 0221
            << 0321 << 0240 << 061  << 0302 << 0113 << 0300 << 073
            << 0245 << 0301 << 0364 << 0363 << 0141 << 0140 << 0173
            << 0174 << 0175 << 0176 << 020  << 021  << 0250 << 01
            << 02   << 03   << 04   << 05   << 06   << 011  << 012
            << 013  << 014  << 015  << 016  << 017  << 022  << 023
            << 024  << 025  << 026  << 027  << 030  << 032  ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! из КСС в КСУ
    arTable.setCh(&(t->chTable.ch[AR_IN_KSU_UCOKS1]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 10, 0);
    arTable << 02   << 04   << 034  << 05   << 0171 << 0172
            << 055  << 06   << 07   << 01   << 026  << 0121
            << 057  << 0220 << 03   << 010  << 011;
    //    arTable << 034  << 05   << 0171 << 0172 << 026  << 055  << 0121
    //            << 057  << 0220 << 06   << 07   << 01   << 02   << 03
    //            << 04   << 010  << 011 ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! из КСС в КСУ
    arTable.setCh(&(t->chTable.ch[AR_IN_KSU_UCOKS2]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 7, 0);
    arTable << 01   << 02   << 04   << 034  << 05   << 0171 << 0172
            << 055  << 06   << 07   << 026  << 0121 << 057  << 0220
            << 03   << 010  << 011;
//    arTable << 034  << 05   << 0171 << 0172 << 026  << 055  << 0121
//            << 057  << 0220 << 06   << 07   << 01   << 02   << 03
//            << 04   << 010  << 011 ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! из КСС в КСУ
    arTable.setCh(&(t->chTable.ch[AR_IN_KSU_CIMSS]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 9, 0);
    arTable << 01   << 02   << 04   << 034  << 05   << 0171 << 0172
            << 055  << 06   << 07   << 026  << 0121 << 057  << 0220
            << 03   << 010  << 011;
//    arTable << 034  << 05   << 0171 << 0172 << 026  << 055  << 0121
//            << 057  << 0220 << 06   << 07   << 01   << 02   << 03
//            << 04   << 010  << 011 ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! БКС1 в КСУ
    arTable.setCh(&(t->chTable.ch[AR_OUT_BKS1_KSU]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 9, 0);
    arTable << 310;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! БКС2 в КСУ
    arTable.setCh(&(t->chTable.ch[AR_OUT_BKS2_KSU]));
    arTable.setAddr(E_NODE_PV, 3);
    arTable.setChId(E_CH_AR, 9, 0);
    arTable << 310 ;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_BKS3_KSS]));
    arTable.setAddr(E_NODE_PV, 1);
    arTable.setChId(E_CH_AR, 2, 1);
    arTable << 0100 << 0101 << 0102 << 0103 << 0104 << 0105 << 0106
            << 0107 << 0110 << 0111 << 0112 << 0113 << 0114 << 0115
            << 0116 << 0117 << 0120 << 0121 << 0122 << 0123 << 0124
            << 0125 << 0126 << 0127 << 0130;

    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);


    t->chTable.ch[RV_OTK].typeNode           = E_NODE_PV;
    t->chTable.ch[RV_OTK].idNode             = 1;
    t->chTable.ch[RV_OTK].typeCh             = E_CH_RK;
    t->chTable.ch[RV_OTK].idCh               = 0;
    t->chTable.ch[RV_OTK].setting.numAdapter = 1;
    t->chTable.ch[RV_OTK].setting.numCh      = 26;
    t->chTable.ch[RV_OTK].setting.ioCh       = 1;

    t->chTable.ch[RV_VKL].typeNode           = E_NODE_PV;
    t->chTable.ch[RV_VKL].idNode             = 1;
    t->chTable.ch[RV_VKL].typeCh             = E_CH_RK;
    t->chTable.ch[RV_VKL].idCh               = 0;
    t->chTable.ch[RV_VKL].setting.numAdapter = 1;
    t->chTable.ch[RV_VKL].setting.numCh      = 27;
    t->chTable.ch[RV_VKL].setting.ioCh       = 1;

//    //Вкл вентилятора ЭВ1
//    t->chTable.ch[SVO_EV1_BKS1].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV1_BKS1].idNode             = 1;
//    t->chTable.ch[SVO_EV1_BKS1].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV1_BKS1].idCh               = 0;
//    t->chTable.ch[SVO_EV1_BKS1].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV1_BKS1].setting.numCh      = 48 + 29;
//    t->chTable.ch[SVO_EV1_BKS1].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ2
//    t->chTable.ch[SVO_EV2_BKS4].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV2_BKS4].idNode             = 1;
//    t->chTable.ch[SVO_EV2_BKS4].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV2_BKS4].idCh               = 0;
//    t->chTable.ch[SVO_EV2_BKS4].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV2_BKS4].setting.numCh      = 48 + 30;
//    t->chTable.ch[SVO_EV2_BKS4].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ3
//    t->chTable.ch[SVO_EV3_BKS2].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV3_BKS2].idNode             = 1;
//    t->chTable.ch[SVO_EV3_BKS2].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV3_BKS2].idCh               = 0;
//    t->chTable.ch[SVO_EV3_BKS2].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV3_BKS2].setting.numCh      = 48 + 31;
//    t->chTable.ch[SVO_EV3_BKS2].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ3
//    t->chTable.ch[SVO_EV3_BKS4].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV3_BKS4].idNode             = 1;
//    t->chTable.ch[SVO_EV3_BKS4].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV3_BKS4].idCh               = 0;
//    t->chTable.ch[SVO_EV3_BKS4].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV3_BKS4].setting.numCh      = 48 + 32;
//    t->chTable.ch[SVO_EV3_BKS4].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ4
//    t->chTable.ch[SVO_EV4_BKS2].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV4_BKS2].idNode             = 1;
//    t->chTable.ch[SVO_EV4_BKS2].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV4_BKS2].idCh               = 0;
//    t->chTable.ch[SVO_EV4_BKS2].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV4_BKS2].setting.numCh      = 48 + 33;
//    t->chTable.ch[SVO_EV4_BKS2].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ4
//    t->chTable.ch[SVO_EV4_BKS4].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV4_BKS4].idNode             = 1;
//    t->chTable.ch[SVO_EV4_BKS4].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV4_BKS4].idCh               = 0;
//    t->chTable.ch[SVO_EV4_BKS4].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV4_BKS4].setting.numCh      = 48 + 34;
//    t->chTable.ch[SVO_EV4_BKS4].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ5
//    t->chTable.ch[SVO_EV5_BKS1].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS1].idNode             = 1;
//    t->chTable.ch[SVO_EV5_BKS1].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS1].idCh               = 0;
//    t->chTable.ch[SVO_EV5_BKS1].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV5_BKS1].setting.numCh      = 48 + 35;
//    t->chTable.ch[SVO_EV5_BKS1].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ5
//    t->chTable.ch[SVO_EV5_BKS2].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS2].idNode             = 1;
//    t->chTable.ch[SVO_EV5_BKS2].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS2].idCh               = 0;
//    t->chTable.ch[SVO_EV5_BKS2].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV5_BKS2].setting.numCh      = 48 + 36;
//    t->chTable.ch[SVO_EV5_BKS2].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ5
//    t->chTable.ch[SVO_EV5_BKS3].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS3].idNode             = 1;
//    t->chTable.ch[SVO_EV5_BKS3].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS3].idCh               = 0;
//    t->chTable.ch[SVO_EV5_BKS3].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV5_BKS3].setting.numCh      = 48 + 37;
//    t->chTable.ch[SVO_EV5_BKS3].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ5
//    t->chTable.ch[SVO_EV5_BKS4].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV5_BKS4].idNode             = 1;
//    t->chTable.ch[SVO_EV5_BKS4].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV5_BKS4].idCh               = 0;
//    t->chTable.ch[SVO_EV5_BKS4].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV5_BKS4].setting.numCh      = 48 + 38;
//    t->chTable.ch[SVO_EV5_BKS4].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ6
//    t->chTable.ch[SVO_EV6_BKS1].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS1].idNode             = 1;
//    t->chTable.ch[SVO_EV6_BKS1].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS1].idCh               = 0;
//    t->chTable.ch[SVO_EV6_BKS1].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV6_BKS1].setting.numCh      = 48 + 39;
//    t->chTable.ch[SVO_EV6_BKS1].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ6
//    t->chTable.ch[SVO_EV6_BKS2].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS2].idNode             = 1;
//    t->chTable.ch[SVO_EV6_BKS2].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS2].idCh               = 0;
//    t->chTable.ch[SVO_EV6_BKS2].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV6_BKS2].setting.numCh      = 48 + 40;
//    t->chTable.ch[SVO_EV6_BKS2].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ6
//    t->chTable.ch[SVO_EV6_BKS3].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS3].idNode             = 1;
//    t->chTable.ch[SVO_EV6_BKS3].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS3].idCh               = 0;
//    t->chTable.ch[SVO_EV6_BKS3].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV6_BKS3].setting.numCh      = 48 + 41;
//    t->chTable.ch[SVO_EV6_BKS3].setting.ioCh       = 1;
//
//    //Вкл вентилятора ЭВ6
//    t->chTable.ch[SVO_EV6_BKS4].typeNode           = E_NODE_PV;
//    t->chTable.ch[SVO_EV6_BKS4].idNode             = 1;
//    t->chTable.ch[SVO_EV6_BKS4].typeCh             = E_CH_RK;
//    t->chTable.ch[SVO_EV6_BKS4].idCh               = 0;
//    t->chTable.ch[SVO_EV6_BKS4].setting.numAdapter = 0;
//    t->chTable.ch[SVO_EV6_BKS4].setting.numCh      = 48 + 42;
//    t->chTable.ch[SVO_EV6_BKS4].setting.ioCh       = 1;

    //Вкл вентиляторов колёс
//    t->chTable.ch[182].typeNode           = E_NODE_PV;
//    t->chTable.ch[182].idNode             = 1;
//    t->chTable.ch[182].typeCh             = E_CH_RK;
//    t->chTable.ch[182].idCh               = 0;
//    t->chTable.ch[182].setting.numAdapter = 1;
//    t->chTable.ch[182].setting.numCh      = 0;
//    t->chTable.ch[182].setting.ioCh       = 1;


    t->chTable.ch[SVO_DT1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DT1].idNode             = 1;
    t->chTable.ch[SVO_DT1].typeCh             = E_IR;
    t->chTable.ch[SVO_DT1].idCh               = 0;
    t->chTable.ch[SVO_DT1].setting.numAdapter = 0;
    t->chTable.ch[SVO_DT1].setting.numCh      = 8;
    t->chTable.ch[SVO_DT1].setting.ioCh       = 1;
    ir = (TDesIR*) (t->chTable.ch[SVO_DT1].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    t->chTable.ch[SVO_DT2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DT2].idNode             = 1;
    t->chTable.ch[SVO_DT2].typeCh             = E_IR;
    t->chTable.ch[SVO_DT2].idCh               = 0;
    t->chTable.ch[SVO_DT2].setting.numAdapter = 0;
    t->chTable.ch[SVO_DT2].setting.numCh      = 9;
    t->chTable.ch[SVO_DT2].setting.ioCh       = 1;
    ir = (TDesIR*) (t->chTable.ch[SVO_DT2].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    t->chTable.ch[SVO_DT3].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DT3].idNode             = 1;
    t->chTable.ch[SVO_DT3].typeCh             = E_IR;
    t->chTable.ch[SVO_DT3].idCh               = 0;
    t->chTable.ch[SVO_DT3].setting.numAdapter = 0;
    t->chTable.ch[SVO_DT3].setting.numCh      = 0;
    t->chTable.ch[SVO_DT3].setting.ioCh       = 1;
    ir = (TDesIR*) (t->chTable.ch[SVO_DT3].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    t->chTable.ch[SVO_DT4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DT4].idNode             = 1;
    t->chTable.ch[SVO_DT4].typeCh             = E_IR;
    t->chTable.ch[SVO_DT4].idCh               = 0;
    t->chTable.ch[SVO_DT4].setting.numAdapter = 0;
    t->chTable.ch[SVO_DT4].setting.numCh      = 1;
    t->chTable.ch[SVO_DT4].setting.ioCh       = 1;
    ir = (TDesIR*) (t->chTable.ch[SVO_DT4].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    t->chTable.ch[SVO_DT5].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DT5].idNode             = 1;
    t->chTable.ch[SVO_DT5].typeCh             = E_IR;
    t->chTable.ch[SVO_DT5].idCh               = 0;
    t->chTable.ch[SVO_DT5].setting.numAdapter = 0;
    t->chTable.ch[SVO_DT5].setting.numCh      = 12 + 0;
    t->chTable.ch[SVO_DT5].setting.ioCh       = 1;
    ir = (TDesIR*) (t->chTable.ch[SVO_DT5].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    t->chTable.ch[SVO_DT6].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DT6].idNode             = 1;
    t->chTable.ch[SVO_DT6].typeCh             = E_IR;
    t->chTable.ch[SVO_DT6].idCh               = 0;
    t->chTable.ch[SVO_DT6].setting.numAdapter = 0;
    t->chTable.ch[SVO_DT6].setting.numCh      = 12 + 1;
    t->chTable.ch[SVO_DT6].setting.ioCh       = 1;
    ir = (TDesIR*) (t->chTable.ch[SVO_DT6].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    t->chTable.ch[SVO_DT7].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DT7].idNode             = 1;
    t->chTable.ch[SVO_DT7].typeCh             = E_IR;
    t->chTable.ch[SVO_DT7].idCh               = 0;
    t->chTable.ch[SVO_DT7].setting.numAdapter = 0;
    t->chTable.ch[SVO_DT7].setting.numCh      = 12 + 2;
    t->chTable.ch[SVO_DT7].setting.ioCh       = 1;
    ir = (TDesIR*) (t->chTable.ch[SVO_DT7].desData);
    ir->max = 240;
    ir->min = 30;
    ir->value = ir->min;

    t->chTable.ch[SVO_DD_BCVM_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BCVM_BKS1].idNode             = 1;
    t->chTable.ch[SVO_DD_BCVM_BKS1].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BCVM_BKS1].idCh               = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS1].setting.numCh      = 8;
    t->chTable.ch[SVO_DD_BCVM_BKS1].setting.ioCh       = 1;

    t->chTable.ch[SVO_DD_BCVM_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BCVM_BKS2].idNode             = 1;
    t->chTable.ch[SVO_DD_BCVM_BKS2].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BCVM_BKS2].idCh               = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS2].setting.numCh      = 10;
    t->chTable.ch[SVO_DD_BCVM_BKS2].setting.ioCh       = 1;

    t->chTable.ch[SVO_DD_BCVM_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BCVM_BKS3].idNode             = 1;
    t->chTable.ch[SVO_DD_BCVM_BKS3].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BCVM_BKS3].idCh               = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS3].setting.numCh      = 9;
    t->chTable.ch[SVO_DD_BCVM_BKS3].setting.ioCh       = 1;

    t->chTable.ch[SVO_DD_BCVM_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BCVM_BKS4].idNode             = 1;
    t->chTable.ch[SVO_DD_BCVM_BKS4].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BCVM_BKS4].idCh               = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BCVM_BKS4].setting.numCh      = 11;
    t->chTable.ch[SVO_DD_BCVM_BKS4].setting.ioCh       = 1;

    t->chTable.ch[SVO_DD_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BKS1].idNode             = 1;
    t->chTable.ch[SVO_DD_BKS1].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BKS1].idCh               = 0;
    t->chTable.ch[SVO_DD_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BKS1].setting.numCh      = 12;
    t->chTable.ch[SVO_DD_BKS1].setting.ioCh       = 1;

    t->chTable.ch[SVO_DD_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BKS2].idNode             = 1;
    t->chTable.ch[SVO_DD_BKS2].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BKS2].idCh               = 0;
    t->chTable.ch[SVO_DD_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BKS2].setting.numCh      = 13;
    t->chTable.ch[SVO_DD_BKS2].setting.ioCh       = 1;

    t->chTable.ch[SVO_DD_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BKS3].idNode             = 1;
    t->chTable.ch[SVO_DD_BKS3].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BKS3].idCh               = 0;
    t->chTable.ch[SVO_DD_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BKS3].setting.numCh      = 14;
    t->chTable.ch[SVO_DD_BKS3].setting.ioCh       = 1;

    t->chTable.ch[SVO_DD_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_DD_BKS4].idNode             = 1;
    t->chTable.ch[SVO_DD_BKS4].typeCh             = E_CH_DAC;
    t->chTable.ch[SVO_DD_BKS4].idCh               = 0;
    t->chTable.ch[SVO_DD_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_DD_BKS4].setting.numCh      = 15;
    t->chTable.ch[SVO_DD_BKS4].setting.ioCh       = 1;

    t->chTable.ch[SVO_COM_EV1_27V_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV1_27V_BKS1].idNode             = 1;
    t->chTable.ch[SVO_COM_EV1_27V_BKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV1_27V_BKS1].idCh               = 0;
    t->chTable.ch[SVO_COM_EV1_27V_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV1_27V_BKS1].setting.numCh      = 29;
    t->chTable.ch[SVO_COM_EV1_27V_BKS1].setting.ioCh       = 0;


    t->chTable.ch[SVO_COM_EV2_27V_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV2_27V_BKS4].idNode             = 1;
    t->chTable.ch[SVO_COM_EV2_27V_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV2_27V_BKS4].idCh               = 0;
    t->chTable.ch[SVO_COM_EV2_27V_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV2_27V_BKS4].setting.numCh      = 30;
    t->chTable.ch[SVO_COM_EV2_27V_BKS4].setting.ioCh       = 0;


    t->chTable.ch[SVO_COM_EV3_27V_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV3_27V_BKS2].idNode             = 1;
    t->chTable.ch[SVO_COM_EV3_27V_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV3_27V_BKS2].idCh               = 0;
    t->chTable.ch[SVO_COM_EV3_27V_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV3_27V_BKS2].setting.numCh      = 31;
    t->chTable.ch[SVO_COM_EV3_27V_BKS2].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV3_27V_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV3_27V_BKS4].idNode             = 1;
    t->chTable.ch[SVO_COM_EV3_27V_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV3_27V_BKS4].idCh               = 0;
    t->chTable.ch[SVO_COM_EV3_27V_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV3_27V_BKS4].setting.numCh      = 32;
    t->chTable.ch[SVO_COM_EV3_27V_BKS4].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV4_27V_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV4_27V_BKS2].idNode             = 1;
    t->chTable.ch[SVO_COM_EV4_27V_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV4_27V_BKS2].idCh               = 0;
    t->chTable.ch[SVO_COM_EV4_27V_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV4_27V_BKS2].setting.numCh      = 33;
    t->chTable.ch[SVO_COM_EV4_27V_BKS2].setting.ioCh       = 0;


    t->chTable.ch[SVO_COM_EV4_27V_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV4_27V_BKS4].idNode             = 1;
    t->chTable.ch[SVO_COM_EV4_27V_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV4_27V_BKS4].idCh               = 0;
    t->chTable.ch[SVO_COM_EV4_27V_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV4_27V_BKS4].setting.numCh      = 34;
    t->chTable.ch[SVO_COM_EV4_27V_BKS4].setting.ioCh       = 0;


    t->chTable.ch[SVO_COM_EV5_27V_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV5_27V_BKS1].idNode             = 1;
    t->chTable.ch[SVO_COM_EV5_27V_BKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV5_27V_BKS1].idCh               = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS1].setting.numCh      = 35;
    t->chTable.ch[SVO_COM_EV5_27V_BKS1].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV5_27V_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV5_27V_BKS2].idNode             = 1;
    t->chTable.ch[SVO_COM_EV5_27V_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV5_27V_BKS2].idCh               = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS2].setting.numCh      = 36;
    t->chTable.ch[SVO_COM_EV5_27V_BKS2].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV5_27V_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV5_27V_BKS3].idNode             = 1;
    t->chTable.ch[SVO_COM_EV5_27V_BKS3].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV5_27V_BKS3].idCh               = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS3].setting.numCh      = 37;
    t->chTable.ch[SVO_COM_EV5_27V_BKS3].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV5_27V_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV5_27V_BKS4].idNode             = 1;
    t->chTable.ch[SVO_COM_EV5_27V_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV5_27V_BKS4].idCh               = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV5_27V_BKS4].setting.numCh      = 38;
    t->chTable.ch[SVO_COM_EV5_27V_BKS4].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV6_27V_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV6_27V_BKS1].idNode             = 1;
    t->chTable.ch[SVO_COM_EV6_27V_BKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV6_27V_BKS1].idCh               = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS1].setting.numCh      = 39;
    t->chTable.ch[SVO_COM_EV6_27V_BKS1].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV6_27V_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV6_27V_BKS2].idNode             = 1;
    t->chTable.ch[SVO_COM_EV6_27V_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV6_27V_BKS2].idCh               = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS2].setting.numCh      = 40;
    t->chTable.ch[SVO_COM_EV6_27V_BKS2].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV6_27V_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV6_27V_BKS3].idNode             = 1;
    t->chTable.ch[SVO_COM_EV6_27V_BKS3].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV6_27V_BKS3].idCh               = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS3].setting.numCh      = 41;
    t->chTable.ch[SVO_COM_EV6_27V_BKS3].setting.ioCh       = 0;

    t->chTable.ch[SVO_COM_EV6_27V_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_COM_EV6_27V_BKS4].idNode             = 1;
    t->chTable.ch[SVO_COM_EV6_27V_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_COM_EV6_27V_BKS4].idCh               = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_COM_EV6_27V_BKS4].setting.numCh      = 42;
    t->chTable.ch[SVO_COM_EV6_27V_BKS4].setting.ioCh       = 0;

    //////
    t->chTable.ch[SVO_OUT_EV1_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV1_BKS1].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV1_BKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV1_BKS1].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV1_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV1_BKS1].setting.numCh      = 1;
    t->chTable.ch[SVO_OUT_EV1_BKS1].setting.ioCh       = 1;


    t->chTable.ch[SVO_OUT_EV2_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV2_BKS4].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV2_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV2_BKS4].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV2_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV2_BKS4].setting.numCh      = 2;
    t->chTable.ch[SVO_OUT_EV2_BKS4].setting.ioCh       = 1;


    t->chTable.ch[SVO_OUT_EV3_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV3_BKS2].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV3_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV3_BKS2].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV3_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV3_BKS2].setting.numCh      = 3;
    t->chTable.ch[SVO_OUT_EV3_BKS2].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV3_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV3_BKS4].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV3_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV3_BKS4].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV3_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV3_BKS4].setting.numCh      = 4;
    t->chTable.ch[SVO_OUT_EV3_BKS4].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV4_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV4_BKS2].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV4_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV4_BKS2].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV4_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV4_BKS2].setting.numCh      = 5;
    t->chTable.ch[SVO_OUT_EV4_BKS2].setting.ioCh       = 1;


    t->chTable.ch[SVO_OUT_EV4_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV4_BKS4].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV4_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV4_BKS4].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV4_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV4_BKS4].setting.numCh      = 6;
    t->chTable.ch[SVO_OUT_EV4_BKS4].setting.ioCh       = 1;


    t->chTable.ch[SVO_OUT_EV5_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV5_BKS1].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV5_BKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV5_BKS1].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS1].setting.numCh      = 7;
    t->chTable.ch[SVO_OUT_EV5_BKS1].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV5_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV5_BKS2].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV5_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV5_BKS2].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS2].setting.numCh      = 8;
    t->chTable.ch[SVO_OUT_EV5_BKS2].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV5_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV5_BKS3].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV5_BKS3].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV5_BKS3].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS3].setting.numCh      = 9;
    t->chTable.ch[SVO_OUT_EV5_BKS3].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV5_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV5_BKS4].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV5_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV5_BKS4].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV5_BKS4].setting.numCh      = 38;
    t->chTable.ch[SVO_OUT_EV5_BKS4].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV6_BKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV6_BKS1].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV6_BKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV6_BKS1].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS1].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS1].setting.numCh      = 10;
    t->chTable.ch[SVO_OUT_EV6_BKS1].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV6_BKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV6_BKS2].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV6_BKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV6_BKS2].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS2].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS2].setting.numCh      = 11;
    t->chTable.ch[SVO_OUT_EV6_BKS2].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV6_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV6_BKS3].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV6_BKS3].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV6_BKS3].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS3].setting.numCh      = 12;
    t->chTable.ch[SVO_OUT_EV6_BKS3].setting.ioCh       = 1;

    t->chTable.ch[SVO_OUT_EV6_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SVO_OUT_EV6_BKS4].idNode             = 1;
    t->chTable.ch[SVO_OUT_EV6_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SVO_OUT_EV6_BKS4].idCh               = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SVO_OUT_EV6_BKS4].setting.numCh      = 13;
    t->chTable.ch[SVO_OUT_EV6_BKS4].setting.ioCh       = 1;
////

    t->chTable.ch[SPZ_FIRE_ENG_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_FIRE_ENG_BKS3].idNode             = 3;
    t->chTable.ch[SPZ_FIRE_ENG_BKS3].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_FIRE_ENG_BKS3].idCh               = 0;
    t->chTable.ch[SPZ_FIRE_ENG_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SPZ_FIRE_ENG_BKS3].setting.numCh      = 13;//47;
    t->chTable.ch[SPZ_FIRE_ENG_BKS3].setting.ioCh       = 1;

//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_1].typeNode           = E_NODE_PV;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_1].idNode             = 3;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_1].typeCh             = E_CH_RK;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_1].idCh               = 0;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_1].setting.numAdapter = 0;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_1].setting.numCh      = 5;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_1].setting.ioCh       = 1;
//
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_2].typeNode           = E_NODE_PV;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_2].idNode             = 3;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_2].typeCh             = E_CH_RK;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_2].idCh               = 0;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_2].setting.numAdapter = 0;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_2].setting.numCh      = 5;
//    t->chTable.ch[SPZ_FIRE_ENG_UCOKS_2].setting.ioCh       = 1;

    t->chTable.ch[SPZ_FIRE_ENG_CIMSS_A].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_FIRE_ENG_CIMSS_A].idNode             = 3;
    t->chTable.ch[SPZ_FIRE_ENG_CIMSS_A].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_FIRE_ENG_CIMSS_A].idCh               = 0;
    t->chTable.ch[SPZ_FIRE_ENG_CIMSS_A].setting.numAdapter = 0;
    t->chTable.ch[SPZ_FIRE_ENG_CIMSS_A].setting.numCh      = 5;
    t->chTable.ch[SPZ_FIRE_ENG_CIMSS_A].setting.ioCh       = 1;

//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_1].typeNode           = E_NODE_PV;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_1].idNode             = 3;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_1].typeCh             = E_CH_RK;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_1].idCh               = 0;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_1].setting.numAdapter = 0;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_1].setting.numCh      = 6;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_1].setting.ioCh       = 1;
//
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_2].typeNode           = E_NODE_PV;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_2].idNode             = 3;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_2].typeCh             = E_CH_RK;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_2].idCh               = 0;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_2].setting.numAdapter = 0;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_2].setting.numCh      = 6;
//    t->chTable.ch[SPZ_FIRE_VSU_UCOKS_2].setting.ioCh       = 1;

    t->chTable.ch[SPZ_FIRE_VSU_CIMSS_A].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_FIRE_VSU_CIMSS_A].idNode             = 3;
    t->chTable.ch[SPZ_FIRE_VSU_CIMSS_A].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_FIRE_VSU_CIMSS_A].idCh               = 0;
    t->chTable.ch[SPZ_FIRE_VSU_CIMSS_A].setting.numAdapter = 0;
    t->chTable.ch[SPZ_FIRE_VSU_CIMSS_A].setting.numCh      = 6;
    t->chTable.ch[SPZ_FIRE_VSU_CIMSS_A].setting.ioCh       = 1;

    t->chTable.ch[SPZ_FIRE_VSU_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_FIRE_VSU_BKS4].idNode             = 3;
    t->chTable.ch[SPZ_FIRE_VSU_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_FIRE_VSU_BKS4].idCh               = 0;
    t->chTable.ch[SPZ_FIRE_VSU_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SPZ_FIRE_VSU_BKS4].setting.numCh      = 17;
    t->chTable.ch[SPZ_FIRE_VSU_BKS4].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PK_ENG_CLOSED].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PK_ENG_CLOSED].idNode             = 3;
    t->chTable.ch[SPZ_PK_ENG_CLOSED].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PK_ENG_CLOSED].idCh               = 0;
    t->chTable.ch[SPZ_PK_ENG_CLOSED].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PK_ENG_CLOSED].setting.numCh      = 7;
    t->chTable.ch[SPZ_PK_ENG_CLOSED].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PK_ENG_OPENED].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PK_ENG_OPENED].idNode             = 3;
    t->chTable.ch[SPZ_PK_ENG_OPENED].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PK_ENG_OPENED].idCh               = 0;
    t->chTable.ch[SPZ_PK_ENG_OPENED].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PK_ENG_OPENED].setting.numCh      = 8;
    t->chTable.ch[SPZ_PK_ENG_OPENED].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PK_OHL_ZAKR].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PK_OHL_ZAKR].idNode             = 3;
    t->chTable.ch[SPZ_PK_OHL_ZAKR].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PK_OHL_ZAKR].idCh               = 0;
    t->chTable.ch[SPZ_PK_OHL_ZAKR].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PK_OHL_ZAKR].setting.numCh      = 9;
    t->chTable.ch[SPZ_PK_OHL_ZAKR].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PK_OHL_OTKR].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PK_OHL_OTKR].idNode             = 3;
    t->chTable.ch[SPZ_PK_OHL_OTKR].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PK_OHL_OTKR].idCh               = 0;
    t->chTable.ch[SPZ_PK_OHL_OTKR].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PK_OHL_OTKR].setting.numCh      = 10;
    t->chTable.ch[SPZ_PK_OHL_OTKR].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PK_VSU_CLOSED].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PK_VSU_CLOSED].idNode             = 3;
    t->chTable.ch[SPZ_PK_VSU_CLOSED].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PK_VSU_CLOSED].idCh               = 0;
    t->chTable.ch[SPZ_PK_VSU_CLOSED].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PK_VSU_CLOSED].setting.numCh      = 11;
    t->chTable.ch[SPZ_PK_VSU_CLOSED].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PK_VSU_OPENED].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PK_VSU_OPENED].idNode             = 3;
    t->chTable.ch[SPZ_PK_VSU_OPENED].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PK_VSU_OPENED].idCh               = 0;
    t->chTable.ch[SPZ_PK_VSU_OPENED].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PK_VSU_OPENED].setting.numCh      = 12;
    t->chTable.ch[SPZ_PK_VSU_OPENED].setting.ioCh       = 1;

//    t->chTable.ch[SPZ_FIRE_ENG_KARP].typeNode           = E_NODE_PV; // SPZ_FIRE_ENG_KARP=SPZ_FIRE_ENG_BKS3
//    t->chTable.ch[SPZ_FIRE_ENG_KARP].idNode             = 3;
//    t->chTable.ch[SPZ_FIRE_ENG_KARP].typeCh             = E_CH_RK;
//    t->chTable.ch[SPZ_FIRE_ENG_KARP].idCh               = 0;
//    t->chTable.ch[SPZ_FIRE_ENG_KARP].setting.numAdapter = 0;
//    t->chTable.ch[SPZ_FIRE_ENG_KARP].setting.numCh      = 13;
//    t->chTable.ch[SPZ_FIRE_ENG_KARP].setting.ioCh       = 1;

    t->chTable.ch[SPZ_GA1_DISARMED_BKS3].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_GA1_DISARMED_BKS3].idNode             = 3;
    t->chTable.ch[SPZ_GA1_DISARMED_BKS3].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_GA1_DISARMED_BKS3].idCh               = 0;
    t->chTable.ch[SPZ_GA1_DISARMED_BKS3].setting.numAdapter = 0;
    t->chTable.ch[SPZ_GA1_DISARMED_BKS3].setting.numCh      = 16;
    t->chTable.ch[SPZ_GA1_DISARMED_BKS3].setting.ioCh       = 1;

//    t->chTable.ch[SPZ_FIRE_VSU_KARP].typeNode           = E_NODE_PV; // SPZ_FIRE_VSU_KARP=SPZ_FIRE_VSU_BKS4
//    t->chTable.ch[SPZ_FIRE_VSU_KARP].idNode             = 3;
//    t->chTable.ch[SPZ_FIRE_VSU_KARP].typeCh             = E_CH_RK;
//    t->chTable.ch[SPZ_FIRE_VSU_KARP].idCh               = 0;
//    t->chTable.ch[SPZ_FIRE_VSU_KARP].setting.numAdapter = 0;
//    t->chTable.ch[SPZ_FIRE_VSU_KARP].setting.numCh      = 17;
//    t->chTable.ch[SPZ_FIRE_VSU_KARP].setting.ioCh       = 1;

//    t->chTable.ch[SPZ_CONTRSPS].typeNode           = E_NODE_PV;
//    t->chTable.ch[SPZ_CONTRSPS].idNode             = 3;
//    t->chTable.ch[SPZ_CONTRSPS].typeCh             = E_CH_RK;
//    t->chTable.ch[SPZ_CONTRSPS].idCh               = 0;
//    t->chTable.ch[SPZ_CONTRSPS].setting.numAdapter = 0;
//    t->chTable.ch[SPZ_CONTRSPS].setting.numCh      = 17;
//    t->chTable.ch[SPZ_CONTRSPS].setting.ioCh       = 1;


    t->chTable.ch[SPZ_GA2_DISARMED_BKS4].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_GA2_DISARMED_BKS4].idNode             = 3;
    t->chTable.ch[SPZ_GA2_DISARMED_BKS4].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_GA2_DISARMED_BKS4].idCh               = 0;
    t->chTable.ch[SPZ_GA2_DISARMED_BKS4].setting.numAdapter = 0;
    t->chTable.ch[SPZ_GA2_DISARMED_BKS4].setting.numCh      = 18;
    t->chTable.ch[SPZ_GA2_DISARMED_BKS4].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PRESS_FIRE_ENGINE].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_FIRE_ENGINE].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_FIRE_ENGINE].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_FIRE_ENGINE].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_FIRE_ENGINE].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_FIRE_ENGINE].setting.numCh      = 14;
    t->chTable.ch[SPZ_PRESS_FIRE_ENGINE].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PRESS_FIRE_VSU].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_FIRE_VSU].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_FIRE_VSU].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_FIRE_VSU].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_FIRE_VSU].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_FIRE_VSU].setting.numCh      = 15;
    t->chTable.ch[SPZ_PRESS_FIRE_VSU].setting.ioCh       = 1;

    t->chTable.ch[SPZ_PRESS_ENG_CIMSS_A].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_ENG_CIMSS_A].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_ENG_CIMSS_A].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_ENG_CIMSS_A].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_ENG_CIMSS_A].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_ENG_CIMSS_A].setting.numCh      = 21;
    t->chTable.ch[SPZ_PRESS_ENG_CIMSS_A].setting.ioCh       = 0;

    t->chTable.ch[SPZ_PRESS_ENG_UCOKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS1].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS1].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS1].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS1].setting.numCh      = 22;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS1].setting.ioCh       = 0;

    t->chTable.ch[SPZ_PRESS_ENG_UCOKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS2].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS2].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS2].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS2].setting.numCh      = 23;
    t->chTable.ch[SPZ_PRESS_ENG_UCOKS2].setting.ioCh       = 0;

    t->chTable.ch[SPZ_PRESS_VSU_CIMSS_A].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_VSU_CIMSS_A].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_VSU_CIMSS_A].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_VSU_CIMSS_A].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_VSU_CIMSS_A].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_VSU_CIMSS_A].setting.numCh      = 24;
    t->chTable.ch[SPZ_PRESS_VSU_CIMSS_A].setting.ioCh       = 0;

    t->chTable.ch[SPZ_PRESS_VSU_UCOKS1].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS1].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS1].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS1].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS1].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS1].setting.numCh      = 25;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS1].setting.ioCh       = 0;

    t->chTable.ch[SPZ_PRESS_VSU_UCOKS2].typeNode           = E_NODE_PV;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS2].idNode             = 3;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS2].typeCh             = E_CH_RK;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS2].idCh               = 0;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS2].setting.numAdapter = 0;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS2].setting.numCh      = 26;
    t->chTable.ch[SPZ_PRESS_VSU_UCOKS2].setting.ioCh       = 0;

    t->chTable.ch[SVS1_RK_ISPR].typeNode           = E_NODE_PV;
    t->chTable.ch[SVS1_RK_ISPR].idNode             = 3;
    t->chTable.ch[SVS1_RK_ISPR].typeCh             = E_CH_RK;
    t->chTable.ch[SVS1_RK_ISPR].idCh               = 0;
    t->chTable.ch[SVS1_RK_ISPR].setting.numAdapter = 0;
    t->chTable.ch[SVS1_RK_ISPR].setting.numCh      = 44;
    t->chTable.ch[SVS1_RK_ISPR].setting.ioCh       = 1;

    t->chTable.ch[SVS2_RK_ISPR].typeNode           = E_NODE_PV;
    t->chTable.ch[SVS2_RK_ISPR].idNode             = 3;
    t->chTable.ch[SVS2_RK_ISPR].typeCh             = E_CH_RK;
    t->chTable.ch[SVS2_RK_ISPR].idCh               = 0;
    t->chTable.ch[SVS2_RK_ISPR].setting.numAdapter = 0;
    t->chTable.ch[SVS2_RK_ISPR].setting.numCh      = 45;
    t->chTable.ch[SVS2_RK_ISPR].setting.ioCh       = 1;

    t->chTable.ch[SVS3_RK_ISPR].typeNode           = E_NODE_PV;
    t->chTable.ch[SVS3_RK_ISPR].idNode             = 3;
    t->chTable.ch[SVS3_RK_ISPR].typeCh             = E_CH_RK;
    t->chTable.ch[SVS3_RK_ISPR].idCh               = 0;
    t->chTable.ch[SVS3_RK_ISPR].setting.numAdapter = 0;
    t->chTable.ch[SVS3_RK_ISPR].setting.numCh      = 46;
    t->chTable.ch[SVS3_RK_ISPR].setting.ioCh       = 1;

    t->chTable.ch[APDDVIM1_OUT_VIBOR_VX].typeNode           = E_NODE_PV;
    t->chTable.ch[APDDVIM1_OUT_VIBOR_VX].idNode             = 3;
    t->chTable.ch[APDDVIM1_OUT_VIBOR_VX].typeCh             = E_CH_RK;
    t->chTable.ch[APDDVIM1_OUT_VIBOR_VX].idCh               = 0;
    t->chTable.ch[APDDVIM1_OUT_VIBOR_VX].setting.numAdapter = 0;
    t->chTable.ch[APDDVIM1_OUT_VIBOR_VX].setting.numCh      = 3;
    t->chTable.ch[APDDVIM1_OUT_VIBOR_VX].setting.ioCh       = 1;

    t->chTable.ch[APDDVIM2_OUT_VIBOR_VX].typeNode           = E_NODE_PV;
    t->chTable.ch[APDDVIM2_OUT_VIBOR_VX].idNode             = 3;
    t->chTable.ch[APDDVIM2_OUT_VIBOR_VX].typeCh             = E_CH_RK;
    t->chTable.ch[APDDVIM2_OUT_VIBOR_VX].idCh               = 0;
    t->chTable.ch[APDDVIM2_OUT_VIBOR_VX].setting.numAdapter = 0;
    t->chTable.ch[APDDVIM2_OUT_VIBOR_VX].setting.numCh      = 4;
    t->chTable.ch[APDDVIM2_OUT_VIBOR_VX].setting.ioCh       = 1;

    t->chTable.ch[RK_IN_APDD_1_VIBOR_VX].typeNode           = E_NODE_PV;
    t->chTable.ch[RK_IN_APDD_1_VIBOR_VX].idNode             = 3;
    t->chTable.ch[RK_IN_APDD_1_VIBOR_VX].typeCh             = E_CH_RK;
    t->chTable.ch[RK_IN_APDD_1_VIBOR_VX].idCh               = 0;
    t->chTable.ch[RK_IN_APDD_1_VIBOR_VX].setting.numAdapter = 0;
    t->chTable.ch[RK_IN_APDD_1_VIBOR_VX].setting.numCh      = 46;
    t->chTable.ch[RK_IN_APDD_1_VIBOR_VX].setting.ioCh       = 0;
    
    t->chTable.ch[RK_IN_APDD_2_VIBOR_VX].typeNode           = E_NODE_PV;
    t->chTable.ch[RK_IN_APDD_2_VIBOR_VX].idNode             = 3;
    t->chTable.ch[RK_IN_APDD_2_VIBOR_VX].typeCh             = E_CH_RK;
    t->chTable.ch[RK_IN_APDD_2_VIBOR_VX].idCh               = 0;
    t->chTable.ch[RK_IN_APDD_2_VIBOR_VX].setting.numAdapter = 0;
    t->chTable.ch[RK_IN_APDD_2_VIBOR_VX].setting.numCh      = 47;
    t->chTable.ch[RK_IN_APDD_2_VIBOR_VX].setting.ioCh       = 0;

    t->chTable.ch[VSU_ZAPUSK_VSU].typeNode           = E_NODE_PV;
    t->chTable.ch[VSU_ZAPUSK_VSU].idNode             = 1;
    t->chTable.ch[VSU_ZAPUSK_VSU].typeCh             = E_CH_RK;
    t->chTable.ch[VSU_ZAPUSK_VSU].idCh               = 0;
    t->chTable.ch[VSU_ZAPUSK_VSU].setting.numAdapter = 0;
    t->chTable.ch[VSU_ZAPUSK_VSU].setting.numCh      = 44;
    t->chTable.ch[VSU_ZAPUSK_VSU].setting.ioCh       = 0;

    t->chTable.ch[VSU_OSTANOV_VSU].typeNode           = E_NODE_PV;
    t->chTable.ch[VSU_OSTANOV_VSU].idNode             = 4;
    t->chTable.ch[VSU_OSTANOV_VSU].typeCh             = E_CH_RK;
    t->chTable.ch[VSU_OSTANOV_VSU].idCh               = 0;
    t->chTable.ch[VSU_OSTANOV_VSU].setting.numAdapter = 0;
    t->chTable.ch[VSU_OSTANOV_VSU].setting.numCh      = 5;
    t->chTable.ch[VSU_OSTANOV_VSU].setting.ioCh       = 1;

    t->chTable.ch[VSU_ZAPUSK_VSU].typeNode           = E_NODE_PV;
    t->chTable.ch[VSU_ZAPUSK_VSU].idNode             = 4;
    t->chTable.ch[VSU_ZAPUSK_VSU].typeCh             = E_CH_RK;
    t->chTable.ch[VSU_ZAPUSK_VSU].idCh               = 0;
    t->chTable.ch[VSU_ZAPUSK_VSU].setting.numAdapter = 0;
    t->chTable.ch[VSU_ZAPUSK_VSU].setting.numCh      = 6;
    t->chTable.ch[VSU_ZAPUSK_VSU].setting.ioCh       = 1;

    t->chTable.ch[SES_OSN_SHIN].typeNode           = E_NODE_PV;
    t->chTable.ch[SES_OSN_SHIN].idNode             = 1;
    t->chTable.ch[SES_OSN_SHIN].typeCh             = E_CH_DAC;
    t->chTable.ch[SES_OSN_SHIN].idCh               = 0;
    t->chTable.ch[SES_OSN_SHIN].setting.numAdapter = 0;
    t->chTable.ch[SES_OSN_SHIN].setting.numCh      = 16 + 0;
    t->chTable.ch[SES_OSN_SHIN].setting.ioCh       = 1;

    t->chTable.ch[SES_LEV_SHIN1].typeNode           = E_NODE_PV;
    t->chTable.ch[SES_LEV_SHIN1].idNode             = 1;
    t->chTable.ch[SES_LEV_SHIN1].typeCh             = E_CH_DAC;
    t->chTable.ch[SES_LEV_SHIN1].idCh               = 0;
    t->chTable.ch[SES_LEV_SHIN1].setting.numAdapter = 0;
    t->chTable.ch[SES_LEV_SHIN1].setting.numCh      = 16 + 1;
    t->chTable.ch[SES_LEV_SHIN1].setting.ioCh       = 1;

    t->chTable.ch[SES_LEV_SHIN2].typeNode           = E_NODE_PV;
    t->chTable.ch[SES_LEV_SHIN2].idNode             = 1;
    t->chTable.ch[SES_LEV_SHIN2].typeCh             = E_CH_DAC;
    t->chTable.ch[SES_LEV_SHIN2].idCh               = 0;
    t->chTable.ch[SES_LEV_SHIN2].setting.numAdapter = 0;
    t->chTable.ch[SES_LEV_SHIN2].setting.numCh      = 16 + 2;
    t->chTable.ch[SES_LEV_SHIN2].setting.ioCh       = 1;

    t->chTable.ch[SES_PRAV_SHIN1].typeNode           = E_NODE_PV;
    t->chTable.ch[SES_PRAV_SHIN1].idNode             = 1;
    t->chTable.ch[SES_PRAV_SHIN1].typeCh             = E_CH_DAC;
    t->chTable.ch[SES_PRAV_SHIN1].idCh               = 0;
    t->chTable.ch[SES_PRAV_SHIN1].setting.numAdapter = 0;
    t->chTable.ch[SES_PRAV_SHIN1].setting.numCh      = 16 + 3;
    t->chTable.ch[SES_PRAV_SHIN1].setting.ioCh       = 1;

    t->chTable.ch[SES_PRAV_SHIN2].typeNode           = E_NODE_PV;
    t->chTable.ch[SES_PRAV_SHIN2].idNode             = 1;
    t->chTable.ch[SES_PRAV_SHIN2].typeCh             = E_CH_DAC;
    t->chTable.ch[SES_PRAV_SHIN2].idCh               = 0;
    t->chTable.ch[SES_PRAV_SHIN2].setting.numAdapter = 0;
    t->chTable.ch[SES_PRAV_SHIN2].setting.numCh      = 16 + 4;
    t->chTable.ch[SES_PRAV_SHIN2].setting.ioCh       = 1;


    //ЕВ6_БКС2
    t->chTable.ch[KSU_SHASS_VIP_REZ1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_VIP_REZ1].idNode = 3;
    t->chTable.ch[KSU_SHASS_VIP_REZ1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_VIP_REZ1].idCh = 0;
    t->chTable.ch[KSU_SHASS_VIP_REZ1].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_VIP_REZ1].setting.numCh = 11;
    t->chTable.ch[KSU_SHASS_VIP_REZ1].setting.ioCh = 0;


    t->chTable.ch[KSU_SHASS_VIP_REZ2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_VIP_REZ2].idNode = 3;
    t->chTable.ch[KSU_SHASS_VIP_REZ2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_VIP_REZ2].idCh = 0;
    t->chTable.ch[KSU_SHASS_VIP_REZ2].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_VIP_REZ2].setting.numCh = 13;
    t->chTable.ch[KSU_SHASS_VIP_REZ2].setting.ioCh = 0;


    t->chTable.ch[KSU_SHASS_VIP_REZ3].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_VIP_REZ3].idNode = 3;
    t->chTable.ch[KSU_SHASS_VIP_REZ3].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_VIP_REZ3].idCh = 0;
    t->chTable.ch[KSU_SHASS_VIP_REZ3].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_VIP_REZ3].setting.numCh = 15;
    t->chTable.ch[KSU_SHASS_VIP_REZ3].setting.ioCh = 0;

    t->chTable.ch[KSU_SHASS_VIP_REZ4].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_VIP_REZ4].idNode = 3;
    t->chTable.ch[KSU_SHASS_VIP_REZ4].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_VIP_REZ4].idCh = 0;
    t->chTable.ch[KSU_SHASS_VIP_REZ4].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_VIP_REZ4].setting.numCh = 17;
    t->chTable.ch[KSU_SHASS_VIP_REZ4].setting.ioCh = 0;


    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ1].idNode = 3;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ1].idCh = 0;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ1].setting.numAdapter = 1;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ1].setting.numCh = 7;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ1].setting.ioCh = 0;

    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ2].idNode = 3;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ2].idCh = 0;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ2].setting.numAdapter = 1;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ2].setting.numCh = 8;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ2].setting.ioCh = 0;

    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ3].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ3].idNode = 3;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ3].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ3].idCh = 0;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ3].setting.numAdapter = 1;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ3].setting.numCh = 9;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ3].setting.ioCh = 0;

    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ4].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ4].idNode = 3;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ4].typeCh = E_CH_RK;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ4].idCh = 0;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ4].setting.numAdapter = 1;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ4].setting.numCh = 10;
    t->chTable.ch[KSU_AVAR_SHASS_VIP_REZ4].setting.ioCh = 0;


    t->chTable.ch[KSU_SHASS_UBOR_REZ1].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_UBOR_REZ1].idNode = 3;
    t->chTable.ch[KSU_SHASS_UBOR_REZ1].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_UBOR_REZ1].idCh = 0;
    t->chTable.ch[KSU_SHASS_UBOR_REZ1].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_UBOR_REZ1].setting.numCh = 12;
    t->chTable.ch[KSU_SHASS_UBOR_REZ1].setting.ioCh = 0;

    t->chTable.ch[KSU_SHASS_UBOR_REZ2].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_UBOR_REZ2].idNode = 3;
    t->chTable.ch[KSU_SHASS_UBOR_REZ2].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_UBOR_REZ2].idCh = 0;
    t->chTable.ch[KSU_SHASS_UBOR_REZ2].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_UBOR_REZ2].setting.numCh = 13;
    t->chTable.ch[KSU_SHASS_UBOR_REZ2].setting.ioCh = 0;

    t->chTable.ch[KSU_SHASS_UBOR_REZ3].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_UBOR_REZ3].idNode = 3;
    t->chTable.ch[KSU_SHASS_UBOR_REZ3].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_UBOR_REZ3].idCh = 0;
    t->chTable.ch[KSU_SHASS_UBOR_REZ3].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_UBOR_REZ3].setting.numCh = 14;
    t->chTable.ch[KSU_SHASS_UBOR_REZ3].setting.ioCh = 0;

    t->chTable.ch[KSU_SHASS_UBOR_REZ4].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_SHASS_UBOR_REZ4].idNode = 3;
    t->chTable.ch[KSU_SHASS_UBOR_REZ4].typeCh = E_CH_RK;
    t->chTable.ch[KSU_SHASS_UBOR_REZ4].idCh = 0;
    t->chTable.ch[KSU_SHASS_UBOR_REZ4].setting.numAdapter = 1;
    t->chTable.ch[KSU_SHASS_UBOR_REZ4].setting.numCh = 15;
    t->chTable.ch[KSU_SHASS_UBOR_REZ4].setting.ioCh = 0;

    t->chTable.ch[KSU_OUT_VIP_SHASS_OSN].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OUT_VIP_SHASS_OSN].idNode = 3;
    t->chTable.ch[KSU_OUT_VIP_SHASS_OSN].typeCh = E_CH_RK;
    t->chTable.ch[KSU_OUT_VIP_SHASS_OSN].idCh = 0;
    t->chTable.ch[KSU_OUT_VIP_SHASS_OSN].setting.numAdapter = 1;
    t->chTable.ch[KSU_OUT_VIP_SHASS_OSN].setting.numCh = 37;
    t->chTable.ch[KSU_OUT_VIP_SHASS_OSN].setting.ioCh = 1;

    t->chTable.ch[KSU_OUT_UBOR_SHASS_OSN].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OUT_UBOR_SHASS_OSN].idNode = 3;
    t->chTable.ch[KSU_OUT_UBOR_SHASS_OSN].typeCh = E_CH_RK;
    t->chTable.ch[KSU_OUT_UBOR_SHASS_OSN].idCh = 0;
    t->chTable.ch[KSU_OUT_UBOR_SHASS_OSN].setting.numAdapter = 1;
    t->chTable.ch[KSU_OUT_UBOR_SHASS_OSN].setting.numCh = 38;
    t->chTable.ch[KSU_OUT_UBOR_SHASS_OSN].setting.ioCh = 1;

    t->chTable.ch[KSU_OUT_VIP_STV_AVAR].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OUT_VIP_STV_AVAR].idNode = 3;
    t->chTable.ch[KSU_OUT_VIP_STV_AVAR].typeCh = E_CH_RK;
    t->chTable.ch[KSU_OUT_VIP_STV_AVAR].idCh = 0;
    t->chTable.ch[KSU_OUT_VIP_STV_AVAR].setting.numAdapter = 1;
    t->chTable.ch[KSU_OUT_VIP_STV_AVAR].setting.numCh = 39;
    t->chTable.ch[KSU_OUT_VIP_STV_AVAR].setting.ioCh = 1;

    t->chTable.ch[KSU_OUT_VIP_SH_AVAR].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OUT_VIP_SH_AVAR].idNode   = 3;
    t->chTable.ch[KSU_OUT_VIP_SH_AVAR].typeCh   = E_CH_RK;
    t->chTable.ch[KSU_OUT_VIP_SH_AVAR].idCh     = 0;
    t->chTable.ch[KSU_OUT_VIP_SH_AVAR].setting.numAdapter = 1;
    t->chTable.ch[KSU_OUT_VIP_SH_AVAR].setting.numCh      = 40;
    t->chTable.ch[KSU_OUT_VIP_SH_AVAR].setting.ioCh       = 1;

    t->chTable.ch[RK_IN_SUVSH_K_OHL_KOLES].typeNode = E_NODE_PV;
    t->chTable.ch[RK_IN_SUVSH_K_OHL_KOLES].idNode   = 1;
    t->chTable.ch[RK_IN_SUVSH_K_OHL_KOLES].typeCh   = E_CH_RK;
    t->chTable.ch[RK_IN_SUVSH_K_OHL_KOLES].idCh     = 0;
    t->chTable.ch[RK_IN_SUVSH_K_OHL_KOLES].setting.numAdapter = 1;
    t->chTable.ch[RK_IN_SUVSH_K_OHL_KOLES].setting.numCh      = 28;
    t->chTable.ch[RK_IN_SUVSH_K_OHL_KOLES].setting.ioCh       = 0;

    //Сигнал Тест-Контроль в СВС
       t->chTable.ch[SVS1_RK_TEST_CONTR].typeNode = E_NODE_PV;
       t->chTable.ch[SVS1_RK_TEST_CONTR].idNode = 3;
       t->chTable.ch[SVS1_RK_TEST_CONTR].typeCh = E_CH_RK;
       t->chTable.ch[SVS1_RK_TEST_CONTR].idCh = 0;
       t->chTable.ch[SVS1_RK_TEST_CONTR].setting.numAdapter = 1;
    t->chTable.ch[SVS1_RK_TEST_CONTR].setting.numCh = 0;
       t->chTable.ch[SVS1_RK_TEST_CONTR].setting.ioCh = 0;

    //для КСУ
    t->chTable.ch[KSU_VKL_DIST_UPR_1K].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_VKL_DIST_UPR_1K].idNode = 3;
    t->chTable.ch[KSU_VKL_DIST_UPR_1K].typeCh = E_CH_RK;
    t->chTable.ch[KSU_VKL_DIST_UPR_1K].idCh = 0;
    t->chTable.ch[KSU_VKL_DIST_UPR_1K].setting.numAdapter = 0;
    t->chTable.ch[KSU_VKL_DIST_UPR_1K].setting.numCh = 27;
    t->chTable.ch[KSU_VKL_DIST_UPR_1K].setting.ioCh = 0;

    t->chTable.ch[KSU_VKL_DIST_UPR_2K].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_VKL_DIST_UPR_2K].idNode = 3;
    t->chTable.ch[KSU_VKL_DIST_UPR_2K].typeCh = E_CH_RK;
    t->chTable.ch[KSU_VKL_DIST_UPR_2K].idCh = 0;
    t->chTable.ch[KSU_VKL_DIST_UPR_2K].setting.numAdapter = 0;
    t->chTable.ch[KSU_VKL_DIST_UPR_2K].setting.numCh = 28;
    t->chTable.ch[KSU_VKL_DIST_UPR_2K].setting.ioCh = 0;

    t->chTable.ch[KSU_VKL_DIST_UPR_3K].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_VKL_DIST_UPR_3K].idNode = 3;
    t->chTable.ch[KSU_VKL_DIST_UPR_3K].typeCh = E_CH_RK;
    t->chTable.ch[KSU_VKL_DIST_UPR_3K].idCh = 0;
    t->chTable.ch[KSU_VKL_DIST_UPR_3K].setting.numAdapter = 0;
    t->chTable.ch[KSU_VKL_DIST_UPR_3K].setting.numCh = 29;
    t->chTable.ch[KSU_VKL_DIST_UPR_3K].setting.ioCh = 0;

    t->chTable.ch[KSU_VIBOR_DIST_UPR].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_VIBOR_DIST_UPR].idNode = 3;
    t->chTable.ch[KSU_VIBOR_DIST_UPR].typeCh = E_CH_RK;
    t->chTable.ch[KSU_VIBOR_DIST_UPR].idCh = 0;
    t->chTable.ch[KSU_VIBOR_DIST_UPR].setting.numAdapter = 0;
    t->chTable.ch[KSU_VIBOR_DIST_UPR].setting.numCh = 30;
    t->chTable.ch[KSU_VIBOR_DIST_UPR].setting.ioCh = 0;

    t->chTable.ch[KSU_OBOGREV_HALF].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OBOGREV_HALF].idNode = 3;
    t->chTable.ch[KSU_OBOGREV_HALF].typeCh = E_CH_RK;
    t->chTable.ch[KSU_OBOGREV_HALF].idCh = 0;
    t->chTable.ch[KSU_OBOGREV_HALF].setting.numAdapter = 0;
    t->chTable.ch[KSU_OBOGREV_HALF].setting.numCh = 31;
    t->chTable.ch[KSU_OBOGREV_HALF].setting.ioCh = 0;

    t->chTable.ch[KSU_OBOGREV_FULL].typeNode = E_NODE_PV;
    t->chTable.ch[KSU_OBOGREV_FULL].idNode = 3;
    t->chTable.ch[KSU_OBOGREV_FULL].typeCh = E_CH_RK;
    t->chTable.ch[KSU_OBOGREV_FULL].idCh = 0;
    t->chTable.ch[KSU_OBOGREV_FULL].setting.numAdapter = 0;
    t->chTable.ch[KSU_OBOGREV_FULL].setting.numCh = 32;
    t->chTable.ch[KSU_OBOGREV_FULL].setting.ioCh = 0;

    //! БИНС1 1 подадрес
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 1;
    mil->numWord = 27;

    //! БИНС1 2 подадрес
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 2;
    mil->numWord = 21;
    //! БИНС1 3 адрес
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 3;
    mil->numWord = 17;

    //! БИНС1 4 адрес
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 4;
    mil->numWord = 31;

    //! БИНС1 5 адрес
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS1_58_SP2_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 27;
    mil->subAddr = 5;
    mil->numWord = 16;

    //! БИНС1 1 адрес
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].idNode = 3;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS1_58_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 27;
    mil->subAddr = 1;
    mil->numWord = 9;

    //! БИНС1 2 адрес
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].idNode = 3;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS1_58_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 27;
    mil->subAddr = 2;
    mil->numWord = 9;

    ////////////////////////////////////////////////////////////////
    //! БИНС3 1 подадрес
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 1;
    mil->numWord = 26;

    //! БИНС3 2 подадрес
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 2;
    mil->numWord = 21;
    //! БИНС3 3 адрес
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 3;
    mil->numWord = 17;

    //! БИНС3 4 адрес
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 4;
    mil->numWord = 31;

    //! БИНС3 5 адрес
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].idNode = 3;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BINS2_58_SP2_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 28;
    mil->subAddr = 5;
    mil->numWord = 16;

    //! БИНС3 1 адрес
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].idNode = 1;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS2_58_SP2_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 28;
    mil->subAddr = 1;
    mil->numWord = 9;

    //! БИНС3 2 адрес
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].idNode = 1;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BINS2_58_SP2_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 28;
    mil->subAddr = 2;
    mil->numWord = 9;


    //Авар пит вкл
    t->chTable.ch[KSU58_AVAR_PIT].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_AVAR_PIT].idNode               = 5;
    t->chTable.ch[KSU58_AVAR_PIT].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_AVAR_PIT].idCh                 = 0;
    t->chTable.ch[KSU58_AVAR_PIT].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_AVAR_PIT].setting.numCh        = 5;
    t->chTable.ch[KSU58_AVAR_PIT].setting.ioCh         = 1;

    //Аэродр пит вкл
    t->chTable.ch[KSU58_AEROD_PIT].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_AEROD_PIT].idNode               = 5;
    t->chTable.ch[KSU58_AEROD_PIT].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_AEROD_PIT].idCh                 = 0;
    t->chTable.ch[KSU58_AEROD_PIT].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_AEROD_PIT].setting.numCh        = 14;
    t->chTable.ch[KSU58_AEROD_PIT].setting.ioCh         = 1;

    //Откз ГС1
    t->chTable.ch[KSU58_OTKAZ_GS1].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_OTKAZ_GS1].idNode               = 5;
    t->chTable.ch[KSU58_OTKAZ_GS1].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_OTKAZ_GS1].idCh                 = 0;
    t->chTable.ch[KSU58_OTKAZ_GS1].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_OTKAZ_GS1].setting.numCh        = 12;
    t->chTable.ch[KSU58_OTKAZ_GS1].setting.ioCh         = 1;

    //Откз ГС2
    t->chTable.ch[KSU58_OTKAZ_GS2].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_OTKAZ_GS2].idNode               = 5;
    t->chTable.ch[KSU58_OTKAZ_GS2].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_OTKAZ_GS2].idCh                 = 0;
    t->chTable.ch[KSU58_OTKAZ_GS2].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_OTKAZ_GS2].setting.numCh        = 13;
    t->chTable.ch[KSU58_OTKAZ_GS2].setting.ioCh         = 1;

    //Давл в ст ГС
    t->chTable.ch[KSU58_DAV_ST_GS].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_DAV_ST_GS].idNode               = 5;
    t->chTable.ch[KSU58_DAV_ST_GS].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_DAV_ST_GS].idCh                 = 0;
    t->chTable.ch[KSU58_DAV_ST_GS].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_DAV_ST_GS].setting.numCh        = 8;
    t->chTable.ch[KSU58_DAV_ST_GS].setting.ioCh         = 1;

    //ОПС1
    t->chTable.ch[KSU58_OPS_1].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_OPS_1].idNode               = 5;
    t->chTable.ch[KSU58_OPS_1].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_OPS_1].idCh                 = 0;
    t->chTable.ch[KSU58_OPS_1].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_OPS_1].setting.numCh        = 10;
    t->chTable.ch[KSU58_OPS_1].setting.ioCh         = 1;

    //ОПС2
    t->chTable.ch[KSU58_OPS_2].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_OPS_2].idNode               = 5;
    t->chTable.ch[KSU58_OPS_2].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_OPS_2].idCh                 = 0;
    t->chTable.ch[KSU58_OPS_2].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_OPS_2].setting.numCh        = 11;
    t->chTable.ch[KSU58_OPS_2].setting.ioCh         = 1;

    //Обж ОГС пр
    t->chTable.ch[KSU58_OBJ_OGS_PRAV].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_OBJ_OGS_PRAV].idNode               = 5;
    t->chTable.ch[KSU58_OBJ_OGS_PRAV].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_OBJ_OGS_PRAV].idCh                 = 0;
    t->chTable.ch[KSU58_OBJ_OGS_PRAV].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_OBJ_OGS_PRAV].setting.numCh        = 6;
    t->chTable.ch[KSU58_OBJ_OGS_PRAV].setting.ioCh         = 1;

    //Обж ОГС пр
    t->chTable.ch[KSU58_OBJ_OGS_LEV].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_OBJ_OGS_LEV].idNode               = 5;
    t->chTable.ch[KSU58_OBJ_OGS_LEV].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_OBJ_OGS_LEV].idCh                 = 0;
    t->chTable.ch[KSU58_OBJ_OGS_LEV].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_OBJ_OGS_LEV].setting.numCh        = 7;
    t->chTable.ch[KSU58_OBJ_OGS_LEV].setting.ioCh         = 1;

    t->chTable.ch[KSU58_MRK2].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_MRK2].idNode               = 5;
    t->chTable.ch[KSU58_MRK2].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_MRK2].idCh                 = 0;
    t->chTable.ch[KSU58_MRK2].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_MRK2].setting.numCh        = 4;
    t->chTable.ch[KSU58_MRK2].setting.ioCh         = 1;

    t->chTable.ch[KSU58_FORCE_LEV].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_FORCE_LEV].idNode               = 5;
    t->chTable.ch[KSU58_FORCE_LEV].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_FORCE_LEV].idCh                 = 0;
    t->chTable.ch[KSU58_FORCE_LEV].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_FORCE_LEV].setting.numCh        = 2;
    t->chTable.ch[KSU58_FORCE_LEV].setting.ioCh         = 1;

    t->chTable.ch[KSU58_FORCE_PRAV].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_FORCE_PRAV].idNode               = 5;
    t->chTable.ch[KSU58_FORCE_PRAV].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_FORCE_PRAV].idCh                 = 0;
    t->chTable.ch[KSU58_FORCE_PRAV].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_FORCE_PRAV].setting.numCh        = 3;
    t->chTable.ch[KSU58_FORCE_PRAV].setting.ioCh         = 1;

    t->chTable.ch[KSU58_SHO].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_SHO].idNode               = 5;
    t->chTable.ch[KSU58_SHO].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_SHO].idCh                 = 0;
    t->chTable.ch[KSU58_SHO].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_SHO].setting.numCh        = 0;
    t->chTable.ch[KSU58_SHO].setting.ioCh         = 1;

    t->chTable.ch[KSU58_MRK2].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_MRK2].idNode               = 5;
    t->chTable.ch[KSU58_MRK2].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_MRK2].idCh                 = 0;
    t->chTable.ch[KSU58_MRK2].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_MRK2].setting.numCh        = 1;
    t->chTable.ch[KSU58_MRK2].setting.ioCh         = 1;

    t->chTable.ch[KSU58_VIP_PO].typeNode             = E_NODE_PV;
    t->chTable.ch[KSU58_VIP_PO].idNode               = 5;
    t->chTable.ch[KSU58_VIP_PO].typeCh               = E_CH_RK;
    t->chTable.ch[KSU58_VIP_PO].idCh                 = 0;
    t->chTable.ch[KSU58_VIP_PO].setting.numAdapter   = 0;
    t->chTable.ch[KSU58_VIP_PO].setting.numCh        = 9;
    t->chTable.ch[KSU58_VIP_PO].setting.ioCh         = 1;

    //! для ЛЛ
    //!

    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 2;
    mil->numWord = 14;

    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 3;
    mil->numWord = 9;

    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 4;
    mil->numWord = 3;

    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 5;
    mil->numWord = 10;

    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS1_58_IUS_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 12;
    mil->subAddr = 1;
    mil->numWord = 22;

    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].idNode = 3;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS1_58_IUS_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 12;
    mil->subAddr = 1;
    mil->numWord = 5;

    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].idNode = 3;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS1_58_IUS_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 12;
    mil->subAddr = 2;
    mil->numWord = 1;

    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].idNode = 3;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].idCh = 0;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS1_58_IUS_SA3].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 12;
    mil->subAddr = 4;
    mil->numWord = 3;

    ////////////////////////////////////////////////////////////

    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 2;
    mil->numWord = 13;

    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 3;
    mil->numWord = 4;

    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 4;
    mil->numWord = 3;

    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 5;
    mil->numWord = 10;

    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS2_58_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 1;
    mil->numWord = 20;

    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].idNode = 3;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS2_58_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 1;
    mil->subAddr = 1;
    mil->numWord = 2;

    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].idNode = 3;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS2_58_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 1;
    mil->subAddr = 2;
    mil->numWord = 1;

    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].idNode = 3;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS2_58_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 1;
    mil->subAddr = 3;
    mil->numWord = 2;

    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].idNode = 3;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].idCh = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS2_58_NVG_SA4].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 1;
    mil->subAddr = 4;
    mil->numWord = 3;
    ////////////////////////
    ///
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 2;
    mil->numWord = 10;

    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 3;
    mil->numWord = 7;

    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 4;
    mil->numWord = 2;

    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA5].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 5;
    mil->numWord = 10;

    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS3_58_IUS_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 13;
    mil->subAddr = 1;
    mil->numWord = 12;

    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].idNode = 3;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS3_58_IUS_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 13;
    mil->subAddr = 1;
    mil->numWord = 6;

    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].idNode = 3;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS3_58_IUS_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 13;
    mil->subAddr = 2;
    mil->numWord = 1;

    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].idNode = 3;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].idCh = 0;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].setting.numCh = LayerMIL::CH_MIL_IUS_58;
    t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS3_58_IUS_SA3].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 13;
    mil->subAddr = 4;
    mil->numWord = 3;
    ////////////////////////////////
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 2;
    mil->subAddr = 2;
    mil->numWord = 8;

    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 2;
    mil->subAddr = 3;
    mil->numWord = 4;

    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 2;
    mil->subAddr = 5;
    mil->numWord = 10;


    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].idNode = 3;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].setting.ioCh = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_BKS4_58_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 2;
    mil->subAddr = 1;
    mil->numWord = 12;

    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].idNode = 3;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].idCh = 0;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS4_58_NVG_SA1].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 2;
    mil->subAddr = 1;
    mil->numWord = 2;

    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].idNode = 3;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].idCh = 0;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS4_58_NVG_SA2].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 2;
    mil->subAddr = 2;
    mil->numWord = 4;

    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].typeNode = E_NODE_CV;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].idNode = 3;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].typeCh = E_CH_MIL;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].idCh = 0;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].setting.numAdapter = 0;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].setting.numCh = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].setting.ioCh = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_BKS4_58_NVG_SA3].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 2;
    mil->subAddr = 4;
    mil->numWord = 3;

    arTable.setCh(&(t->chTable.ch[AR_OUT_ARK_58]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 5, 1);
    arTable << 0162 << 032 ;

    arTable.setProp(LayerArinc::KBs_12_5, LayerArinc::REV_RTM3,
            LayerArinc::ONE);


    t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].idNode              = 3;
    t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].idCh                = 0;
    t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].setting.numAdapter  = 0;
    t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].setting.ioCh        = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_RSBN_BNP_58_NVG_SA11].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 25;
    mil->subAddr = 11;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA15].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 15;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA17].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 17;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA19].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 19;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA21].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 21;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA23].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 23;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BNP_58_NVG_SA6].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 25;
    mil->subAddr = 6;
    mil->numWord = 0;

    t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].idNode              = 3;
    t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].idCh                = 0;
    t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].setting.numAdapter  = 0;
    t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].setting.ioCh        = 0;
    mil = (TDesMIL*) (t->chTable.ch[MIL_IN_RSBN_BP_58_NVG_SA12].desData);
    mil->typeTrans = LayerMIL::KOU;
    mil->addr = 19;
    mil->subAddr = 12;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA16].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 19;
    mil->subAddr = 16;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].idNode              = 3;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].idCh                = 0;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_RSBN_BP_58_NVG_SA18].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 19;
    mil->subAddr = 18;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].idNode              = 3;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].idCh                = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 7;
    mil->subAddr = 2;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].idNode              = 3;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].idCh                = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR58_NVG7_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 7;
    mil->subAddr = 4;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].idNode              = 3;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].idCh                = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 8;
    mil->subAddr = 2;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].idNode              = 3;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].idCh                = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR58_NVG8_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 8;
    mil->subAddr = 4;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].idNode              = 3;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].idCh                = 0;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR58_BP1_SA2].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 2;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].idNode              = 3;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].idCh                = 0;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_KUTR58_BP1_SA4].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 1;
    mil->subAddr = 4;
    mil->numWord = 0;

    t->chTable.ch[BKO2_ISPR_PVD_L].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_ISPR_PVD_L].idNode               = 1;
    t->chTable.ch[BKO2_ISPR_PVD_L].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_ISPR_PVD_L].idCh                 = 0;
    t->chTable.ch[BKO2_ISPR_PVD_L].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_ISPR_PVD_L].setting.numCh        = 48 + 34;
    t->chTable.ch[BKO2_ISPR_PVD_L].setting.ioCh         = 1;

    t->chTable.ch[BKO2_ISPR_DUAS_L].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_ISPR_DUAS_L].idNode               = 1;
    t->chTable.ch[BKO2_ISPR_DUAS_L].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_ISPR_DUAS_L].idCh                 = 0;
    t->chTable.ch[BKO2_ISPR_DUAS_L].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_ISPR_DUAS_L].setting.numCh        = 48 + 35;
    t->chTable.ch[BKO2_ISPR_DUAS_L].setting.ioCh         = 1;

    t->chTable.ch[BKO2_ISPR_PVD_R].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_ISPR_PVD_R].idNode               = 1;
    t->chTable.ch[BKO2_ISPR_PVD_R].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_ISPR_PVD_R].idCh                 = 0;
    t->chTable.ch[BKO2_ISPR_PVD_R].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_ISPR_PVD_R].setting.numCh        = 48 + 36;
    t->chTable.ch[BKO2_ISPR_PVD_R].setting.ioCh         = 1;

    t->chTable.ch[BKO2_ISPR_DUAS_R].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_ISPR_DUAS_R].idNode               = 1;
    t->chTable.ch[BKO2_ISPR_DUAS_R].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_ISPR_DUAS_R].idCh                 = 0;
    t->chTable.ch[BKO2_ISPR_DUAS_R].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_ISPR_DUAS_R].setting.numCh        = 48 + 37;
    t->chTable.ch[BKO2_ISPR_DUAS_R].setting.ioCh         = 1;

    t->chTable.ch[BKO2_FULL_PVD_L].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_FULL_PVD_L].idNode               = 1;
    t->chTable.ch[BKO2_FULL_PVD_L].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_FULL_PVD_L].idCh                 = 0;
    t->chTable.ch[BKO2_FULL_PVD_L].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_FULL_PVD_L].setting.numCh        = 48 + 38;
    t->chTable.ch[BKO2_FULL_PVD_L].setting.ioCh         = 1;

    t->chTable.ch[BKO2_FULL_DUAS_L].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_FULL_DUAS_L].idNode               = 1;
    t->chTable.ch[BKO2_FULL_DUAS_L].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_FULL_DUAS_L].idCh                 = 0;
    t->chTable.ch[BKO2_FULL_DUAS_L].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_FULL_DUAS_L].setting.numCh        = 48 + 39;
    t->chTable.ch[BKO2_FULL_DUAS_L].setting.ioCh         = 1;

    t->chTable.ch[BKO2_FULL_PVD_R].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_FULL_PVD_R].idNode               = 1;
    t->chTable.ch[BKO2_FULL_PVD_R].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_FULL_PVD_R].idCh                 = 0;
    t->chTable.ch[BKO2_FULL_PVD_R].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_FULL_PVD_R].setting.numCh        = 48 + 40;
    t->chTable.ch[BKO2_FULL_PVD_R].setting.ioCh         = 1;

    t->chTable.ch[BKO2_FULL_DUAS_R].typeNode             = E_NODE_PV;
    t->chTable.ch[BKO2_FULL_DUAS_R].idNode               = 1;
    t->chTable.ch[BKO2_FULL_DUAS_R].typeCh               = E_CH_RK;
    t->chTable.ch[BKO2_FULL_DUAS_R].idCh                 = 0;
    t->chTable.ch[BKO2_FULL_DUAS_R].setting.numAdapter   = 1;
    t->chTable.ch[BKO2_FULL_DUAS_R].setting.numCh        = 48 + 41;
    t->chTable.ch[BKO2_FULL_DUAS_R].setting.ioCh         = 1;

    //ЕВ6_БКС2
    t->chTable.ch[RK_IN_RUCH_SHASS_58].typeNode = E_NODE_PV;
    t->chTable.ch[RK_IN_RUCH_SHASS_58].idNode = 5;
    t->chTable.ch[RK_IN_RUCH_SHASS_58].typeCh = E_CH_RK;
    t->chTable.ch[RK_IN_RUCH_SHASS_58].idCh = 0;
    t->chTable.ch[RK_IN_RUCH_SHASS_58].setting.numAdapter = 0;
    t->chTable.ch[RK_IN_RUCH_SHASS_58].setting.numCh = 7;
    t->chTable.ch[RK_IN_RUCH_SHASS_58].setting.ioCh = 0;

    t->chTable.ch[RK_OUT_PR_STEND_KSS].typeNode = E_NODE_PV;
    t->chTable.ch[RK_OUT_PR_STEND_KSS].idNode = 4;
    t->chTable.ch[RK_OUT_PR_STEND_KSS].typeCh = E_CH_RK;
    t->chTable.ch[RK_OUT_PR_STEND_KSS].idCh = 0;
    t->chTable.ch[RK_OUT_PR_STEND_KSS].setting.numAdapter = 0;
    t->chTable.ch[RK_OUT_PR_STEND_KSS].setting.numCh = 11;
    t->chTable.ch[RK_OUT_PR_STEND_KSS].setting.ioCh = 1;

    t->chTable.ch[RK_OUT_SUVSH_SHASS_UBOR_KSS].typeNode = E_NODE_PV;
    t->chTable.ch[RK_OUT_SUVSH_SHASS_UBOR_KSS].idNode = 4;
    t->chTable.ch[RK_OUT_SUVSH_SHASS_UBOR_KSS].typeCh = E_CH_RK;
    t->chTable.ch[RK_OUT_SUVSH_SHASS_UBOR_KSS].idCh = 0;
    t->chTable.ch[RK_OUT_SUVSH_SHASS_UBOR_KSS].setting.numAdapter = 0;
    t->chTable.ch[RK_OUT_SUVSH_SHASS_UBOR_KSS].setting.numCh = 12;
    t->chTable.ch[RK_OUT_SUVSH_SHASS_UBOR_KSS].setting.ioCh = 1;

    t->chTable.ch[RK_IN_UCOKS1_CAB58].typeNode = E_NODE_PV;
    t->chTable.ch[RK_IN_UCOKS1_CAB58].idNode = 5;
    t->chTable.ch[RK_IN_UCOKS1_CAB58].typeCh = E_CH_RK;
    t->chTable.ch[RK_IN_UCOKS1_CAB58].idCh = 0;
    t->chTable.ch[RK_IN_UCOKS1_CAB58].setting.numAdapter = 0;
    t->chTable.ch[RK_IN_UCOKS1_CAB58].setting.numCh = 5;
    t->chTable.ch[RK_IN_UCOKS1_CAB58].setting.ioCh = 0;

    t->chTable.ch[RK_IN_UCOKS2_CAB58].typeNode = E_NODE_PV;
    t->chTable.ch[RK_IN_UCOKS2_CAB58].idNode = 5;
    t->chTable.ch[RK_IN_UCOKS2_CAB58].typeCh = E_CH_RK;
    t->chTable.ch[RK_IN_UCOKS2_CAB58].idCh = 0;
    t->chTable.ch[RK_IN_UCOKS2_CAB58].setting.numAdapter = 0;
    t->chTable.ch[RK_IN_UCOKS2_CAB58].setting.numCh = 6;
    t->chTable.ch[RK_IN_UCOKS2_CAB58].setting.ioCh = 0;

    t->chTable.ch[RK_IN_AIR_CAB58].typeNode = E_NODE_PV;
    t->chTable.ch[RK_IN_AIR_CAB58].idNode = 5;
    t->chTable.ch[RK_IN_AIR_CAB58].typeCh = E_CH_RK;
    t->chTable.ch[RK_IN_AIR_CAB58].idCh = 0;
    t->chTable.ch[RK_IN_AIR_CAB58].setting.numAdapter = 0;
    t->chTable.ch[RK_IN_AIR_CAB58].setting.numCh = 8;
    t->chTable.ch[RK_IN_AIR_CAB58].setting.ioCh = 0;

    t->chTable.ch[RK_IN_RADIO_CAB58].typeNode = E_NODE_PV;
    t->chTable.ch[RK_IN_RADIO_CAB58].idNode = 5;
    t->chTable.ch[RK_IN_RADIO_CAB58].typeCh = E_CH_RK;
    t->chTable.ch[RK_IN_RADIO_CAB58].idCh = 0;
    t->chTable.ch[RK_IN_RADIO_CAB58].setting.numAdapter = 0;
    t->chTable.ch[RK_IN_RADIO_CAB58].setting.numCh = 9;
    t->chTable.ch[RK_IN_RADIO_CAB58].setting.ioCh = 0;



    arTable.setCh(&(t->chTable.ch[AR_OUT_CO2010]));
    arTable.setAddr(E_NODE_PV, 4);
    arTable.setChId(E_CH_AR, 13, 1);
    arTable << 0350;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! линия СБИ 58 (ЦЛС 1...4)
    arTable.setCh(&(t->chTable.ch[AR_IN_KSU_SBI_58]));
    arTable.setAddr(E_NODE_PV, 5);
    arTable.setChId(E_CH_AR, 2, 0);
    arTable << 0240 << 061  << 0113 << 0300 << 0302 << 055  << 054  << 0133
            << 051  << 052  << 057  << 0272 << 0274 << 05   << 06   << 07
            << 0171 << 0172 << 0173 << 0174 << 0175 << 0176 << 0214 ; //! линия СБИ 58 (ЦЛС 1...4)
//    arTable << 0305 << 0154 << 0155 << 0301 << 0302 << 0215 << 0346; //! линия СБИ 58 (ЦЛС 5...8)

    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! линия СБИ для С70
    arTable.setCh(&(t->chTable.ch[AR_IN_KSU_SBI]));
    arTable.setAddr(E_NODE_PV, 5);
    arTable.setChId(E_CH_AR, 2, 0);
    arTable << 05 << 0171 << 0172 << 055 << 054 << 0133 << 051 << 052; // << 0123;

    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

//    //! линия СБИ для С70
//    arTable.setCh(&(t->chTable.ch[AR_IN_KSU_SBI]));
//    arTable.setAddr(E_NODE_PV, 5);
//    arTable.setChId(E_CH_AR, 2, 0);
//    arTable << 05 << 0171  << 0172 << 0133 << 051 << 052 << 0123 << 055 << 054 ;
//
//    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
//            LayerArinc::ALWAYS);

    //! БИНС 1
    arTable.setCh(&(t->chTable.ch[AR_OUT_BINS1_58]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 14, 1);
    arTable << 0270 << 0310 << 0311 << 0312 << 0313 << 0335 << 0314 << 0317
            << 0320 << 0324 << 0325 << 0334 << 0226 << 0366 << 0367 << 0365
            << 0360 << 0327 << 0330 << 0326 << 0331 << 0333 << 0332 << 0364
            << 0361 << 0315 << 0316 << 0321 << 0322;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

    //! БИНС 1  КСУ
    arTable.setCh(&(t->chTable.ch[AR_OUT_BINS1_KSU_58]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 16 + 3, 1);
    arTable << 0270 << 0310 << 0311 << 0312 << 0313 << 0335 << 0314 << 0317
            << 0320 << 0324 << 0325 << 0334 << 0226 << 0366 << 0367 << 0365
            << 0360 << 0327 << 0330 << 0326 << 0331 << 0333 << 0332 << 0364
            << 0361 << 0315 << 0316 << 0321 << 0322;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
          LayerArinc::ALWAYS);

    arTable.setCh(&(t->chTable.ch[AR_OUT_BINS2_KSU_58]));
       arTable.setAddr(E_NODE_CV, 3);
       arTable.setChId(E_CH_AR, 16 + 4, 1);
       arTable << 0270 << 0310 << 0311 << 0312 << 0313 << 0335 << 0314 << 0317
               << 0320 << 0324 << 0325 << 0334 << 0226 << 0366 << 0367 << 0365
               << 0360 << 0327 << 0330 << 0326 << 0331 << 0333 << 0332 << 0364
               << 0361 << 0315 << 0316 << 0321 << 0322;
       arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
             LayerArinc::ALWAYS);

    //! БИНС 2
    arTable.setCh(&(t->chTable.ch[AR_OUT_BINS2_58]));
    arTable.setAddr(E_NODE_CV, 3);
    arTable.setChId(E_CH_AR, 15, 1);
    arTable << 0270 << 0310 << 0311 << 0312 << 0313 << 0335 << 0314 << 0317
            << 0320 << 0324 << 0325 << 0334 << 0226 << 0366 << 0367 << 0365
            << 0360 << 0327 << 0330 << 0326 << 0331 << 0333 << 0332 << 0364
            << 0361 << 0315 << 0316 << 0321 << 0322;
    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
            LayerArinc::ALWAYS);

   t->chTable.ch[KUTR_PPDK_NET].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_PPDK_NET].idNode               = 1;
    t->chTable.ch[KUTR_PPDK_NET].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_PPDK_NET].idCh                 = 0;
    t->chTable.ch[KUTR_PPDK_NET].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_PPDK_NET].setting.numCh        = 2*48 + 15;
    t->chTable.ch[KUTR_PPDK_NET].setting.ioCh         = 1;

    t->chTable.ch[KUTR_OST_750].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_OST_750].idNode               = 1;
    t->chTable.ch[KUTR_OST_750].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_OST_750].idCh                 = 0;
    t->chTable.ch[KUTR_OST_750].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_OST_750].setting.numCh        = 2 * 48 + 21;
    t->chTable.ch[KUTR_OST_750].setting.ioCh         = 1;

    t->chTable.ch[KUTR_P_ECN73_VSU].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_P_ECN73_VSU].idNode               = 1;
    t->chTable.ch[KUTR_P_ECN73_VSU].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_P_ECN73_VSU].idCh                 = 0;
    t->chTable.ch[KUTR_P_ECN73_VSU].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_P_ECN73_VSU].setting.numCh        = 2 * 48 + 16;
    t->chTable.ch[KUTR_P_ECN73_VSU].setting.ioCh         = 1;

    t->chTable.ch[KUTR_ZASOR_FM_VSU].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_ZASOR_FM_VSU].idNode               = 1;
    t->chTable.ch[KUTR_ZASOR_FM_VSU].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_ZASOR_FM_VSU].idCh                 = 0;
    t->chTable.ch[KUTR_ZASOR_FM_VSU].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_ZASOR_FM_VSU].setting.numCh        = 2 * 48 + 17;
    t->chTable.ch[KUTR_ZASOR_FM_VSU].setting.ioCh         = 1;


    t->chTable.ch[KUTR_ZASOR_FM_KSIHT].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_ZASOR_FM_KSIHT].idNode               = 1;
    t->chTable.ch[KUTR_ZASOR_FM_KSIHT].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_ZASOR_FM_KSIHT].idCh                 = 0;
    t->chTable.ch[KUTR_ZASOR_FM_KSIHT].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_ZASOR_FM_KSIHT].setting.numCh        = 2*48 + 18;
    t->chTable.ch[KUTR_ZASOR_FM_KSIHT].setting.ioCh         = 1;

    t->chTable.ch[KUTR_ZASOR_P_K_586200_LEV].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_LEV].idNode               = 1;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_LEV].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_LEV].idCh                 = 0;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_LEV].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_LEV].setting.numCh        = 2*48 + 19;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_LEV].setting.ioCh         = 1;

    t->chTable.ch[KUTR_ZASOR_P_K_586200_PRAV].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_PRAV].idNode               = 1;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_PRAV].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_PRAV].idCh                 = 0;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_PRAV].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_PRAV].setting.numCh        = 2*48 + 20;
    t->chTable.ch[KUTR_ZASOR_P_K_586200_PRAV].setting.ioCh         = 1;

    t->chTable.ch[KUTR_ISPR_KUTR_3].typeNode             = E_NODE_PV;
    t->chTable.ch[KUTR_ISPR_KUTR_3].idNode               = 1;
    t->chTable.ch[KUTR_ISPR_KUTR_3].typeCh               = E_CH_RK;
    t->chTable.ch[KUTR_ISPR_KUTR_3].idCh                 = 0;
    t->chTable.ch[KUTR_ISPR_KUTR_3].setting.numAdapter   = 1;
    t->chTable.ch[KUTR_ISPR_KUTR_3].setting.numCh        = 2*48 + 22;
    t->chTable.ch[KUTR_ISPR_KUTR_3].setting.ioCh         = 1;


    t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_L_58_S1_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 16;
    mil->subAddr = 1;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_L_58_S2_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 16;
    mil->subAddr = 2;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_L_58_S3_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 16;
    mil->subAddr = 3;
    mil->numWord = 0;


    t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_R_58_S1_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 17;
    mil->subAddr = 1;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 17;
    mil->subAddr = 2;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 17;
    mil->subAddr = 3;
    mil->numWord = 0;
  /////////////////////////////////////////////////////////////////////////

    t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_L_58_S1_BP].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 16;
    mil->subAddr = 1;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_L_58_S2_BP].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 16;
    mil->subAddr = 2;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_L_58_S3_BP].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 16;
    mil->subAddr = 3;
    mil->numWord = 0;


    t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_R_58_S1_BP].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 17;
    mil->subAddr = 1;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_ECR_R_58_S2_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_R_58_S2_BP].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 17;
    mil->subAddr = 2;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].setting.numCh       = LayerMIL::CH_MIL_BP_58;
    t->chTable.ch[MIL_OUT_ECR_R_58_S3_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ECR_R_58_S3_BP].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 17;
    mil->subAddr = 3;
    mil->numWord = 0;

    //////////////////////////////////////
    t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ISRP_58_S1_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 11;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ISRP_58_S2_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 12;
    mil->numWord = 0;

    t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].typeNode            = E_NODE_CV;
    t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].idNode              = 3;
    t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].typeCh              = E_CH_MIL;
    t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].idCh                = 0;
    t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].setting.numAdapter  = 0;
    t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].setting.numCh       = LayerMIL::CH_MIL_NVG_58;
    t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].setting.ioCh        = 1;
    mil = (TDesMIL*) (t->chTable.ch[MIL_OUT_ISRP_58_S3_NVG].desData);
    mil->typeTrans = LayerMIL::OUK;
    mil->addr = 5;
    mil->subAddr = 13;
    mil->numWord = 0;

//    arTable.setCh(&(t->chTable.ch[AR_IN_ARK_58]));
//    arTable.setAddr(E_NODE_CV, 3);
//    arTable.setChId(E_CH_AR, 2, 0);
//    arTable << 0100 << 0101 << 0102 << 0103 << 0104 << 0105 << 0106
//            << 0107 << 0110 << 0111 << 0112 << 0113 << 0114 << 0115
//            << 0116 << 0117 << 0120 << 0121 << 0122 << 0123 << 0124
//            << 0125 << 0126 << 0127 << 0130;

//    arTable.setProp(LayerArinc::KBs_100, LayerArinc::REV_RTM3,
//            LayerArinc::ALWAYS);
    //! описание параметров
    //HAL::obj()->paramTable.num = 530;


//    addParamToTable(P_RV_H,             AR_OUT_RV098_1,   E_P_AR_H29_L12_S0_M4096, 0340);
//    addParamToTable(P_RV_H,             AR_OUT_RV098_2,   E_P_AR_H29_L12_S0_M4096, 0340);
//    addParamToTable(P_RV_H,             MIL_OUT_RV098_1_OSO_OUK,   E_P_MIL_H4_L19_S0_M4096, 2);
//    addParamToTable(P_RV_H,             MIL_OUT_RV098_2_NVG_OUK,   E_P_MIL_H4_L19_S0_M4096, 2);
//    addParamToTable(P_RV_MAT_HRV       ,AR_OUT_RV098_1, E_P_AR_H31_L30_S0_M0, 0340);
//    addParamToTable(P_RV_MAT_HRV       ,AR_OUT_RV098_2, E_P_AR_H31_L30_S0_M0, 0340);
//    addParamToTable(P_RV_PR_ISP_INFO,   MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(4), 1,true);
//    addParamToTable(P_RV_PR_ISP_INFO,   MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(4), 1,true);
//    addParamToTable(P_RV_PR_ISP_RV,     MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(5), 1,true);
//    addParamToTable(P_RV_PR_ISP_RV,     MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(5), 1,true);
//    addParamToTable(P_RV_PR_ISP_PP,     MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(6), 1,true);
//    addParamToTable(P_RV_PR_ISP_PP,     MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(6), 1,true);
//    addParamToTable(P_RV_PR_REG_RAB,    MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(8), 1,true);
//    addParamToTable(P_RV_PR_REG_RAB,    MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(8), 1,true);
//    addParamToTable(P_RV_PR_REG_RM,     MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(9), 1,true);
//    addParamToTable(P_RV_PR_REG_RM,     MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(9), 1,true);
//    addParamToTable(P_RV_PR_REG_CNTRL,  MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(10), 1,true);
//    addParamToTable(P_RV_PR_REG_CNTRL,  MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(10), 1,true);
//    addParamToTable(P_RV_PR_REG_MEMR,   MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(11), 1,true);
//    addParamToTable(P_RV_PR_REG_MEMR,   MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(11), 1,true);
//    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(4), 1,true);
//    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(4), 1,true);
//    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(4), 1,true);
//    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(4), 1,true);
//    addParamToTable(P_RV_PR_REG_RAM_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(5), 1,true);
//    addParamToTable(P_RV_PR_REG_RAM_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(5), 1,true);
//    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(6), 1,true);
//    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(6), 1,true);
//    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(6), 1,true);
//    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(6), 1,true);

    addParamToTable(P_RV_H,             AR_OUT_RV098_1,   E_P_AR_H29_L12_S0_M4096, 0340);
    addParamToTable(P_RV_H,             AR_OUT_RV098_2,   E_P_AR_H29_L12_S0_M4096, 0340);
    addParamToTable(P_RV_H,             MIL_OUT_RV098_1_OSO_OUK,   E_P_MIL_H4_L19_S0_M4096, 2);
    addParamToTable(P_RV_H,             MIL_OUT_RV098_2_NVG_OUK,   E_P_MIL_H4_L19_S0_M4096, 2);
    addParamToTable(P_RV_H,             MIL_OUT_RV098_58_NVG_OUK,   E_P_MIL_H4_L19_S0_M4096, 2);

    addParamToTable(P_RV_HV,             MIL_OUT_RV098_1_OSO_OUK,   E_P_MIL_H4_L15_S1_M64, 3);
    addParamToTable(P_RV_HV,             MIL_OUT_RV098_2_NVG_OUK,   E_P_MIL_H4_L15_S1_M64, 3);
    addParamToTable(P_RV_HV,             MIL_OUT_RV098_58_NVG_OUK,   E_P_MIL_H4_L15_S1_M64, 3);

    addParamToTable(P_RV_SIGMAV,        MIL_OUT_RV098_1_OSO_OUK,   E_P_MIL_H4_L15_S1_M64, 4);
    addParamToTable(P_RV_SIGMAV,        MIL_OUT_RV098_2_NVG_OUK,   E_P_MIL_H4_L15_S1_M64, 4);
    addParamToTable(P_RV_SIGMAV,        MIL_OUT_RV098_58_NVG_OUK,   E_P_MIL_H4_L15_S1_M64, 4);


    addParamToTable(P_RV_MAT_HRV       ,AR_OUT_RV098_1, E_P_AR_H31_L30_S0_M0, 0340);
    addParamToTable(P_RV_MAT_HRV       ,AR_OUT_RV098_2, E_P_AR_H31_L30_S0_M0, 0340);
    addParamToTable(P_RV_PR_ISP_INFO,   MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_ISP_INFO,   MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_ISP_INFO,   MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(4), 1,true);

    addParamToTable(P_RV_PR_ISP_RV,     MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(5), 1,true);
    addParamToTable(P_RV_PR_ISP_RV,     MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(5), 1,true);
    addParamToTable(P_RV_PR_ISP_RV,     MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(5), 1,true);
    addParamToTable(P_RV_PR_ISP_PP,     MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(6), 1,true);
    addParamToTable(P_RV_PR_ISP_PP,     MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(6), 1,true);
    addParamToTable(P_RV_PR_ISP_PP,     MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(6), 1,true);

    addParamToTable(P_RV_PR_ISP_AFU,     MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(7), 1,true);
    addParamToTable(P_RV_PR_ISP_AFU,     MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(7), 1,true);
    addParamToTable(P_RV_PR_ISP_AFU,     MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(7), 1,true);

    addParamToTable(P_RV_PR_REG_RAB,    MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(8), 1,true);

    addParamToTable(P_RV_PR_REG_RAB,    MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(8), 1,true);
    addParamToTable(P_RV_PR_REG_RAB,    MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(8), 1,true);
    addParamToTable(P_RV_PR_REG_RM,     MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(9), 1,true);
    addParamToTable(P_RV_PR_REG_RM,     MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(9), 1,true);
    addParamToTable(P_RV_PR_REG_RM,     MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(9), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL,  MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(10), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL,  MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(10), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL,  MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(10), 1,true);
    addParamToTable(P_RV_PR_REG_MEMR,   MIL_OUT_RV098_1_OSO_OUK,   TO_MIL(11), 1,true);
    addParamToTable(P_RV_PR_REG_MEMR,   MIL_OUT_RV098_2_NVG_OUK,   TO_MIL(11), 1,true);
    addParamToTable(P_RV_PR_REG_MEMR,   MIL_OUT_RV098_58_NVG_OUK,   TO_MIL(11), 1,true);
    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_58_NVG_KOU,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_REG_RAB_BCVM,  MIL_IN_RV098_58_NVG_KOU,   TO_MIL(4), 1,true);
    addParamToTable(P_RV_PR_REG_RAM_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(5), 1,true);
    addParamToTable(P_RV_PR_REG_RAM_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(5), 1,true);
    addParamToTable(P_RV_PR_REG_RAM_BCVM,  MIL_IN_RV098_58_NVG_KOU,   TO_MIL(5), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(6), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(6), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_58_NVG_KOU,   TO_MIL(6), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_1_OSO_KOU,   TO_MIL(6), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_2_NVG_KOU,   TO_MIL(6), 1,true);
    addParamToTable(P_RV_PR_REG_CNTRL_BCVM,  MIL_IN_RV098_58_NVG_KOU,   TO_MIL(6), 1,true);

    addParamToTable(P_SVS_H_ABS , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M16384, 0223);
    addParamToTable(P_SVS_H_ABS , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M16384, 0223);
    addParamToTable(P_SVS_H_OTN , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M16384, 0224);
    addParamToTable(P_SVS_H_OTN , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M16384, 0224);
    addParamToTable(P_SVS_VPR   , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M819_2, 0226);
    addParamToTable(P_SVS_VPR   , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M819_2, 0226);
    addParamToTable(P_SVS_VIST  , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M2365 , 0230);
    addParamToTable(P_SVS_VIST  , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M2365 , 0230);
    addParamToTable(P_SVS_MACH  , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M2_048, 0225);
    addParamToTable(P_SVS_MACH  , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M2_048, 0225);
    addParamToTable(P_SVS_VY    , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M256  , 0232);
    addParamToTable(P_SVS_VY    , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M256  , 0232);
    addParamToTable(P_SVS_TN    , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M512  , 0233);
    addParamToTable(P_SVS_TN    , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M512  , 0233);
    addParamToTable(P_SVS_DHG   , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M4096 , 0335);
    addParamToTable(P_SVS_DHG   , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M4096 , 0335);
    addParamToTable(P_SVS_DHB   , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M16384, 0365);
    addParamToTable(P_SVS_DHB   , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M16384, 0365);
    addParamToTable(P_SVS_DM    , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M2_048, 0363);
    addParamToTable(P_SVS_DM    , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M2_048, 0363);
    addParamToTable(P_SVS_DVPR  , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M819_2, 0361);
    addParamToTable(P_SVS_DVPR  , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M819_2, 0361);
    addParamToTable(P_SVS_HG    , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M4096 , 0221);
    addParamToTable(P_SVS_HG    , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M4096 , 0221);
    addParamToTable(P_SVS_PZ    , AR_OUT_SVS1,   E_P_AR_H30_L9_S1_M409_6, 0341);
    addParamToTable(P_SVS_PZ    , AR_OUT_SVS2,   E_P_AR_H30_L9_S1_M409_6, 0341);
    addParamToTable(P_SVS_KS    , AR_OUT_SVS1,   E_P_AR_H31_L9_S0_M0    , 0231);
    addParamToTable(P_SVS_KS    , AR_OUT_SVS2,   E_P_AR_H31_L9_S0_M0    , 0231);

    addParamToTable(P_SVS_DOST_H_ABS    ,     AR_OUT_SVS1,   TO_AR(31), 0223, true);
    addParamToTable(P_SVS_DOST_H_ABS    ,     AR_OUT_SVS2,   TO_AR(31), 0223, true);
    addParamToTable(P_SVS_DOST_H_OTN    ,     AR_OUT_SVS1,   TO_AR(31), 0224, true);
    addParamToTable(P_SVS_DOST_H_OTN    ,     AR_OUT_SVS2,   TO_AR(31), 0224, true);
    addParamToTable(P_SVS_DOST_VPR      ,     AR_OUT_SVS1,   TO_AR(31), 0226, true);
    addParamToTable(P_SVS_DOST_VPR      ,     AR_OUT_SVS2,   TO_AR(31), 0226, true);
    addParamToTable(P_SVS_DOST_VIST     ,     AR_OUT_SVS1,   TO_AR(31), 0230, true);
    addParamToTable(P_SVS_DOST_VIST     ,     AR_OUT_SVS2,   TO_AR(31), 0230, true);
    addParamToTable(P_SVS_DOST_MACH     ,     AR_OUT_SVS1,   TO_AR(31), 0225, true);
    addParamToTable(P_SVS_DOST_MACH     ,     AR_OUT_SVS2,   TO_AR(31), 0225, true);
    addParamToTable(P_SVS_DOST_VY       ,     AR_OUT_SVS1,   TO_AR(31), 0232, true);
    addParamToTable(P_SVS_DOST_VY       ,     AR_OUT_SVS2,   TO_AR(31), 0232, true);
    addParamToTable(P_SVS_DOST_TN       ,     AR_OUT_SVS1,   TO_AR(31), 0233, true);
    addParamToTable(P_SVS_DOST_TN       ,     AR_OUT_SVS2,   TO_AR(31), 0233, true);
    addParamToTable(P_SVS_DOST_DHG      ,     AR_OUT_SVS1,   TO_AR(31), 0335, true);
    addParamToTable(P_SVS_DOST_DHG      ,     AR_OUT_SVS2,   TO_AR(31), 0335, true);
    addParamToTable(P_SVS_DOST_DHB      ,     AR_OUT_SVS1,   TO_AR(31), 0365, true);
    addParamToTable(P_SVS_DOST_DHB      ,     AR_OUT_SVS2,   TO_AR(31), 0365, true);
    addParamToTable(P_SVS_DOST_DM       ,     AR_OUT_SVS1,   TO_AR(31), 0363, true);
    addParamToTable(P_SVS_DOST_DM       ,     AR_OUT_SVS2,   TO_AR(31), 0363, true);
    addParamToTable(P_SVS_DOST_DVPR     ,     AR_OUT_SVS1,   TO_AR(31), 0361, true);
    addParamToTable(P_SVS_DOST_DVPR     ,     AR_OUT_SVS2,   TO_AR(31), 0361, true);
    addParamToTable(P_SVS_DOST_HG       ,     AR_OUT_SVS1,   TO_AR(31), 0221, true);
    addParamToTable(P_SVS_DOST_HG       ,     AR_OUT_SVS2,   TO_AR(31), 0221, true);
    addParamToTable(P_SVS_DOST_PZ       ,     AR_OUT_SVS1,   TO_AR(31), 0341, true);
    addParamToTable(P_SVS_DOST_PZ       ,     AR_OUT_SVS2,   TO_AR(31), 0341, true);



       
    addParamToTable(P_BINS_SS1_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 2);

    addParamToTable(P_BINS_ROLL_I_B1,     MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 10);
    addParamToTable(P_BINS_ROLL_I_B1,     MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 10);
    addParamToTable(P_BINS_ROLL_I_B1,     MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 10);
    addParamToTable(P_BINS_PITCH_I_B1,     MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 11);
    addParamToTable(P_BINS_PITCH_I_B1,     MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 11);
    addParamToTable(P_BINS_PITCH_I_B1,     MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 11);
    addParamToTable(P_BINS_HDG_I_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 12);
    addParamToTable(P_BINS_HDG_I_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 12);
    addParamToTable(P_BINS_HDG_I_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 12);

    addParamToTable(P_BINS_AX_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 4);
    addParamToTable(P_BINS_AX_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 4);
    addParamToTable(P_BINS_AX_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 4);

    addParamToTable(P_BINS_AZ_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 5);
    addParamToTable(P_BINS_AZ_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 5);
    addParamToTable(P_BINS_AZ_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 5);

    addParamToTable(P_BINS_AY_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 6);
    addParamToTable(P_BINS_AY_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 6);
    addParamToTable(P_BINS_AY_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 6);

    addParamToTable(P_BINS_OMEGA_X_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 7);
    addParamToTable(P_BINS_OMEGA_X_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 7);
    addParamToTable(P_BINS_OMEGA_X_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 7);

    addParamToTable(P_BINS_OMEGA_Z_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 8);
    addParamToTable(P_BINS_OMEGA_Z_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 8);
    addParamToTable(P_BINS_OMEGA_Z_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 8);

    addParamToTable(P_BINS_OMEGA_Y_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 9);
    addParamToTable(P_BINS_OMEGA_Y_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 9);
    addParamToTable(P_BINS_OMEGA_Y_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 9);


    addParamToTable(P_BINS_VNI_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 13);
    addParamToTable(P_BINS_VNI_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 13);
    addParamToTable(P_BINS_VNI_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 13);

    addParamToTable(P_BINS_VEI_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 15);
    addParamToTable(P_BINS_VEI_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 15);
    addParamToTable(P_BINS_VEI_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 15);

    addParamToTable(P_BINS_VHBI_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 17);
    addParamToTable(P_BINS_VHBI_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 17);
    addParamToTable(P_BINS_VHBI_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 17);

    addParamToTable(P_BINS_BI_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 19);
    addParamToTable(P_BINS_BI_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 19);
    addParamToTable(P_BINS_BI_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 19);

    addParamToTable(P_BINS_LI_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 21);
    addParamToTable(P_BINS_LI_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 21);
    addParamToTable(P_BINS_LI_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 21);

    addParamToTable(P_BINS_HBI_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M16384, 23);
    addParamToTable(P_BINS_HBI_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M16384, 23);
    addParamToTable(P_BINS_HBI_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M16384, 23);

    addParamToTable(P_BINS_HDGM_I_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 24);
    addParamToTable(P_BINS_HDGM_I_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 24);
    addParamToTable(P_BINS_HDGM_I_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 24);

    addParamToTable(P_BINS_DELTA_M_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S1_MMPI, 25);
    addParamToTable(P_BINS_DELTA_M_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S1_MMPI, 25);
    addParamToTable(P_BINS_DELTA_M_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S1_MMPI, 25);

    addParamToTable(P_BINS_T_WORK_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S0_M32760, 26);
    addParamToTable(P_BINS_T_WORK_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S0_M32760, 26);
    addParamToTable(P_BINS_T_WORK_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S0_M32760, 26);

    addParamToTable(P_BINS_TZAD1_B1,       MIL_OUT_BINS1_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_TZAD1_B1,       MIL_OUT_BINS2_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_TZAD1_B1,       MIL_OUT_BINS3_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 1);

    addParamToTable(P_BINS_SS2_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_SS2_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_SS2_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 1);

       
    addParamToTable(P_BINS_SD2_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SD2_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SD2_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 2);

    addParamToTable(P_BINS_VNSNS1_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 3);
    addParamToTable(P_BINS_VNSNS1_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 3);
    addParamToTable(P_BINS_VNSNS1_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 3);

    addParamToTable(P_BINS_VESNS1_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 5);
    addParamToTable(P_BINS_VESNS1_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 5);
    addParamToTable(P_BINS_VESNS1_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 5);

    addParamToTable(P_BINS_VHSNS1_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 7);
    addParamToTable(P_BINS_VHSNS1_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 7);
    addParamToTable(P_BINS_VHSNS1_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 7);

    addParamToTable(P_BINS_BSNS_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 9);
    addParamToTable(P_BINS_BSNS_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 9);
    addParamToTable(P_BINS_BSNS_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 9);

    addParamToTable(P_BINS_LSNS_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 11);
    addParamToTable(P_BINS_LSNS_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 11);
    addParamToTable(P_BINS_LSNS_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 11);

    addParamToTable(P_BINS_HSNS_B1_MC1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 13);
    addParamToTable(P_BINS_HSNS_B1_MC1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 13);
    addParamToTable(P_BINS_HSNS_B1_MC1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 13);
    //!!
    addParamToTable(P_BINS_TSNS_H_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 14);
    addParamToTable(P_BINS_TSNS_H_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 14);
    addParamToTable(P_BINS_TSNS_H_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 14);

    addParamToTable(P_BINS_TSNS_M_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H9_L14_S0_M0, 14);
    addParamToTable(P_BINS_TSNS_M_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H9_L14_S0_M0, 14);
    addParamToTable(P_BINS_TSNS_M_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H9_L14_S0_M0, 14);

    addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);
    addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);
    addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);

    addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);
    addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);
    addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);

    addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);
    addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);
    addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);

    addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);
    addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);
    addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);

    addParamToTable(P_BINS_DATSNS,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 18);
    addParamToTable(P_BINS_DATSNS,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 18);
    addParamToTable(P_BINS_DATSNS,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 18);

    addParamToTable(P_BINS_GF_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S0_M3276_8, 19);
    addParamToTable(P_BINS_GF_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S0_M3276_8, 19);
    addParamToTable(P_BINS_GF_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S0_M3276_8, 19);

    addParamToTable(P_BINS_NLGN_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_BINS_NLGN_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_BINS_NLGN_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);

    addParamToTable(P_BINS_NGPS_B1,       MIL_OUT_BINS1_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_BINS_NGPS_B1,       MIL_OUT_BINS2_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_BINS_NGPS_B1,       MIL_OUT_BINS3_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);


    addParamToTable(P_BINS_SS1_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 2);

    addParamToTable(P_BINS_ROLL_K_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 4);
    addParamToTable(P_BINS_ROLL_K_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 4);
    addParamToTable(P_BINS_ROLL_K_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 4);

    addParamToTable(P_BINS_PITCH_K_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 5);
    addParamToTable(P_BINS_PITCH_K_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 5);
    addParamToTable(P_BINS_PITCH_K_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 5);

    addParamToTable(P_BINS_HDG_K_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 6);
    addParamToTable(P_BINS_HDG_K_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 6);
    addParamToTable(P_BINS_HDG_K_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 6);

    addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);
    addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);
    addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);

    addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);
    addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);
    addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);

    addParamToTable(P_BINS_VNK_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 8);
    addParamToTable(P_BINS_VNK_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 8);
    addParamToTable(P_BINS_VNK_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 8);

    addParamToTable(P_BINS_VEK_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 10);
    addParamToTable(P_BINS_VEK_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 10);
    addParamToTable(P_BINS_VEK_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 10);


    addParamToTable(P_BINS_VHK_B1_MC1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 12);
    addParamToTable(P_BINS_VHK_B1_MC1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 12);
    addParamToTable(P_BINS_VHK_B1_MC1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 12);

    addParamToTable(P_BINS_BK_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 13);
    addParamToTable(P_BINS_BK_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 13);
    addParamToTable(P_BINS_BK_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 13);


    addParamToTable(P_BINS_LK_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 15);
    addParamToTable(P_BINS_LK_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 15);
    addParamToTable(P_BINS_LK_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 15);

    addParamToTable(P_BINS_HK_B1_MC1D1,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S1_M16384, 17);
    addParamToTable(P_BINS_HK_B1_MC1D1,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S1_M16384, 17);
    addParamToTable(P_BINS_HK_B1_MC1D1,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S1_M16384, 17);


    addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 1);

    addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS1_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS2_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS3_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 1);

    addParamToTable(P_BINS_T_ZAD2_B1_MC1,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_BINS_T_ZAD2_B1_MC1,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_BINS_T_ZAD2_B1_MC1,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 25);

    addParamToTable(P_BINS_SS1_B1_MC1D3,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D3,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D3,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 2);

    addParamToTable(P_BINS_ROLL_I_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 4);
    addParamToTable(P_BINS_ROLL_I_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 4);
    addParamToTable(P_BINS_ROLL_I_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 4);

    addParamToTable(P_BINS_PITCH_I_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 5);
    addParamToTable(P_BINS_PITCH_I_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 5);
    addParamToTable(P_BINS_PITCH_I_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 5);

    addParamToTable(P_BINS_HDG_I_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 6);
    addParamToTable(P_BINS_HDG_I_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 6);
    addParamToTable(P_BINS_HDG_I_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 6);

    addParamToTable(P_BINS_VNI_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 7);
    addParamToTable(P_BINS_VNI_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 7);
    addParamToTable(P_BINS_VNI_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 7);

    addParamToTable(P_BINS_VEI_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 9);
    addParamToTable(P_BINS_VEI_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 9);
    addParamToTable(P_BINS_VEI_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 9);

    addParamToTable(P_BINS_VHBI_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 11);
    addParamToTable(P_BINS_VHBI_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 11);
    addParamToTable(P_BINS_VHBI_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 11);

    addParamToTable(P_BINS_OMEGA_X_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 13);
    addParamToTable(P_BINS_OMEGA_X_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 13);
    addParamToTable(P_BINS_OMEGA_X_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 13);

    addParamToTable(P_BINS_OMEGA_Z_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 14);
    addParamToTable(P_BINS_OMEGA_Z_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 14);
    addParamToTable(P_BINS_OMEGA_Z_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 14);

    addParamToTable(P_BINS_OMEGA_Y_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 15);
    addParamToTable(P_BINS_OMEGA_Y_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 15);
    addParamToTable(P_BINS_OMEGA_Y_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 15);



    /////////////////////////////////////////////////


    addParamToTable(P_BINS_ROLL_K_B1_MC1D2,   MIL_OUT_BINS1_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 16);
    addParamToTable(P_BINS_ROLL_K_B1_MC1D2,   MIL_OUT_BINS2_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 16);
    addParamToTable(P_BINS_ROLL_K_B1_MC1D2,   MIL_OUT_BINS3_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 16);
    addParamToTable(P_BINS_PITCH_K_B1_MC1D2,  MIL_OUT_BINS1_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 17);
    addParamToTable(P_BINS_PITCH_K_B1_MC1D2,  MIL_OUT_BINS2_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 17);
    addParamToTable(P_BINS_PITCH_K_B1_MC1D2,  MIL_OUT_BINS3_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 17);
    addParamToTable(P_BINS_HDG_K_B1_MC1D2,    MIL_OUT_BINS1_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 18);
    addParamToTable(P_BINS_HDG_K_B1_MC1D2,    MIL_OUT_BINS2_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 18);
    addParamToTable(P_BINS_HDG_K_B1_MC1D2,    MIL_OUT_BINS3_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 18);
    addParamToTable(P_BINS_VNK_B1_MC1D2,      MIL_OUT_BINS1_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 19);
    addParamToTable(P_BINS_VNK_B1_MC1D2,      MIL_OUT_BINS2_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 19);
    addParamToTable(P_BINS_VNK_B1_MC1D2,      MIL_OUT_BINS3_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 19);
    addParamToTable(P_BINS_VEK_B1_MC1D2,      MIL_OUT_BINS1_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 21);
    addParamToTable(P_BINS_VEK_B1_MC1D2,      MIL_OUT_BINS2_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 21);
    addParamToTable(P_BINS_VEK_B1_MC1D2,      MIL_OUT_BINS3_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 21);
    addParamToTable(P_BINS_VHBIK_B1_MC1D2,    MIL_OUT_BINS1_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 23);
    addParamToTable(P_BINS_VHBIK_B1_MC1D2,    MIL_OUT_BINS2_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 23);
    addParamToTable(P_BINS_VHBIK_B1_MC1D2,    MIL_OUT_BINS3_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 23);
    addParamToTable(P_BINS_BK_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 26);
    addParamToTable(P_BINS_BK_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 26);
    addParamToTable(P_BINS_BK_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 26);
    addParamToTable(P_BINS_LK_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 28);
    addParamToTable(P_BINS_LK_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 28);
    addParamToTable(P_BINS_LK_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 28);
    addParamToTable(P_BINS_HK_B1_MC1D2,       MIL_OUT_BINS1_SP2_SA3, E_P_MIL_H4_L19_S1_M16384, 30);
    addParamToTable(P_BINS_HK_B1_MC1D2,       MIL_OUT_BINS2_SP2_SA3, E_P_MIL_H4_L19_S1_M16384, 30);
    addParamToTable(P_BINS_HK_B1_MC1D2,       MIL_OUT_BINS3_SP2_SA3, E_P_MIL_H4_L19_S1_M16384, 30);
    addParamToTable(P_BINS_SKO_B1_MC1,        MIL_OUT_BINS1_SP2_SA3, E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_BINS_SKO_B1_MC1,        MIL_OUT_BINS2_SP2_SA3, E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_BINS_SKO_B1_MC1,        MIL_OUT_BINS3_SP2_SA3, E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_BINS_SS1_B1_MC1D4,      MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D4,      MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_SS1_B1_MC1D4,      MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_WK_B1_MC1,         MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 3);
    addParamToTable(P_BINS_WK_B1_MC1,         MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 3);
    addParamToTable(P_BINS_WK_B1_MC1,         MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 3);
    addParamToTable(P_BINS_PU_B1_MC1,         MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 4);
    addParamToTable(P_BINS_PU_B1_MC1,         MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 4);
    addParamToTable(P_BINS_PU_B1_MC1,         MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 4);
    addParamToTable(P_BINS_PUM_B1_MC1,        MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 5);
    addParamToTable(P_BINS_PUM_B1_MC1,        MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 5);
    addParamToTable(P_BINS_PUM_B1_MC1,        MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 5);
    addParamToTable(P_BINS_ATR_B1_MC1,        MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 6);
    addParamToTable(P_BINS_ATR_B1_MC1,        MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 6);
    addParamToTable(P_BINS_ATR_B1_MC1,        MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 6);
    addParamToTable(P_BINS_AG_B1_MC1,         MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_BINS_AG_B1_MC1,         MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_BINS_AG_B1_MC1,         MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_BINS_UG_TR_B1_MC1,      MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 8);
    addParamToTable(P_BINS_UG_TR_B1_MC1,      MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 8);
    addParamToTable(P_BINS_UG_TR_B1_MC1,      MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 8);
    addParamToTable(P_BINS_VY_POT_B1_MC1,     MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_M842_865, 9);
    addParamToTable(P_BINS_VY_POT_B1_MC1,     MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_M842_865, 9);
    addParamToTable(P_BINS_VY_POT_B1_MC1,     MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_M842_865, 9);
    addParamToTable(P_BINS_U_B1_MC1,          MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 10);
    addParamToTable(P_BINS_U_B1_MC1,          MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 10);
    addParamToTable(P_BINS_U_B1_MC1,          MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 10);
    addParamToTable(P_BINS_UG_V_B1_MC1,       MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 11);
    addParamToTable(P_BINS_UG_V_B1_MC1,       MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 11);
    addParamToTable(P_BINS_UG_V_B1_MC1,       MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 11);
    addParamToTable(P_BINS_OMEGA_PU_B1_MC1,   MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_M256, 12);
    addParamToTable(P_BINS_OMEGA_PU_B1_MC1,   MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_M256, 12);
    addParamToTable(P_BINS_OMEGA_PU_B1_MC1,   MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_M256, 12);
    addParamToTable(P_BINS_US_B1_MC1,         MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 13);
    addParamToTable(P_BINS_US_B1_MC1,         MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 13);
    addParamToTable(P_BINS_US_B1_MC1,         MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 13);
    addParamToTable(P_BINS_UG_PLF_B1_MC1,     MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 14);
    addParamToTable(P_BINS_UG_PLF_B1_MC1,     MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 14);
    addParamToTable(P_BINS_UG_PLF_B1_MC1,     MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 14);
    addParamToTable(P_BINS_NPMO_B1_MC1,       MIL_OUT_BINS1_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 15);
    addParamToTable(P_BINS_NPMO_B1_MC1,       MIL_OUT_BINS2_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 15);
    addParamToTable(P_BINS_NPMO_B1_MC1,       MIL_OUT_BINS3_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 15);
    addParamToTable(P_BINS_DOST_USKOR_B1,     MIL_OUT_BINS1_SP2_SA1, TO_MIL(4), 3,true);
    addParamToTable(P_BINS_DOST_USKOR_B1,     MIL_OUT_BINS2_SP2_SA1, TO_MIL(4), 3,true);
    addParamToTable(P_BINS_DOST_USKOR_B1,     MIL_OUT_BINS3_SP2_SA1, TO_MIL(4), 3,true);
    addParamToTable(P_BINS_DOST_OMEGA_B1,     MIL_OUT_BINS1_SP2_SA1, TO_MIL(5), 3,true);
    addParamToTable(P_BINS_DOST_OMEGA_B1,     MIL_OUT_BINS2_SP2_SA1, TO_MIL(5), 3,true);
    addParamToTable(P_BINS_DOST_OMEGA_B1,     MIL_OUT_BINS3_SP2_SA1, TO_MIL(5), 3,true);
    addParamToTable(P_BINS_DOST_VERTIK_I_B1,  MIL_OUT_BINS1_SP2_SA1, TO_MIL(6), 3,true);
    addParamToTable(P_BINS_DOST_VERTIK_I_B1,  MIL_OUT_BINS2_SP2_SA1, TO_MIL(6), 3,true);
    addParamToTable(P_BINS_DOST_VERTIK_I_B1,  MIL_OUT_BINS3_SP2_SA1, TO_MIL(6), 3,true);
    addParamToTable(P_BINS_DOST_HDG_I_B1,     MIL_OUT_BINS1_SP2_SA1, TO_MIL(7), 3,true);
    addParamToTable(P_BINS_DOST_HDG_I_B1,     MIL_OUT_BINS2_SP2_SA1, TO_MIL(7), 3,true);
    addParamToTable(P_BINS_DOST_HDG_I_B1,     MIL_OUT_BINS3_SP2_SA1, TO_MIL(7), 3,true);
    addParamToTable(P_BINS_DOST_HDGM_I_B1,    MIL_OUT_BINS1_SP2_SA1, TO_MIL(8), 3,true);
    addParamToTable(P_BINS_DOST_HDGM_I_B1,    MIL_OUT_BINS2_SP2_SA1, TO_MIL(8), 3,true);
    addParamToTable(P_BINS_DOST_HDGM_I_B1,    MIL_OUT_BINS3_SP2_SA1, TO_MIL(8), 3,true);
    addParamToTable(P_BINS_DOST_DELTA_M_B1,   MIL_OUT_BINS1_SP2_SA1, TO_MIL(9), 3,true);
    addParamToTable(P_BINS_DOST_DELTA_M_B1,   MIL_OUT_BINS2_SP2_SA1, TO_MIL(9), 3,true);
    addParamToTable(P_BINS_DOST_DELTA_M_B1,   MIL_OUT_BINS3_SP2_SA1, TO_MIL(9), 3,true);
    addParamToTable(P_BINS_DOST_SKOR_I_B1,    MIL_OUT_BINS1_SP2_SA1, TO_MIL(10), 3,true);
    addParamToTable(P_BINS_DOST_SKOR_I_B1,    MIL_OUT_BINS2_SP2_SA1, TO_MIL(10), 3,true);
    addParamToTable(P_BINS_DOST_SKOR_I_B1,    MIL_OUT_BINS3_SP2_SA1, TO_MIL(10), 3,true);
    addParamToTable(P_BINS_DOST_KOORD_I_B1,   MIL_OUT_BINS1_SP2_SA1, TO_MIL(11), 3,true);
    addParamToTable(P_BINS_DOST_KOORD_I_B1,   MIL_OUT_BINS2_SP2_SA1, TO_MIL(11), 3,true);
    addParamToTable(P_BINS_DOST_KOORD_I_B1,   MIL_OUT_BINS3_SP2_SA1, TO_MIL(11), 3,true);
    addParamToTable(P_BINS_DOST_HBI_B1,       MIL_OUT_BINS1_SP2_SA1, TO_MIL(12), 3,true);
    addParamToTable(P_BINS_DOST_HBI_B1,       MIL_OUT_BINS2_SP2_SA1, TO_MIL(12), 3,true);
    addParamToTable(P_BINS_DOST_HBI_B1,       MIL_OUT_BINS3_SP2_SA1, TO_MIL(12), 3,true);
    addParamToTable(P_BINS_DOST_TR_B1,        MIL_OUT_BINS1_SP2_SA1, TO_MIL(13), 3,true);
    addParamToTable(P_BINS_DOST_TR_B1,        MIL_OUT_BINS2_SP2_SA1, TO_MIL(13), 3,true);
    addParamToTable(P_BINS_DOST_TR_B1,        MIL_OUT_BINS3_SP2_SA1, TO_MIL(13), 3,true);
    addParamToTable(P_BINS_DOST_H_BAR_B1,        MIL_OUT_BINS1_SP2_SA1, TO_MIL(15), 3,true);
    addParamToTable(P_BINS_DOST_H_BAR_B1,        MIL_OUT_BINS2_SP2_SA1, TO_MIL(15), 3,true);
    addParamToTable(P_BINS_DOST_H_BAR_B1,        MIL_OUT_BINS3_SP2_SA1, TO_MIL(15), 3,true);


   addParamToTable(P_BINS_AR_CCINA              ,-1,  E_P_AR_H29_L14_S0_M0, 0270);
   addParamToTable(P_BINS_AR_ISP_INS            ,-1,  TO_AR(30), 0270, true);
   addParamToTable(P_BINS_AR_SS_B1_BIT_31       ,-1,  TO_AR(31), 0270, true);
   addParamToTable(P_BINS_AR_BK_B1              ,-1,  E_P_AR_H29_L9_S1_M90, 0310);
   addParamToTable(P_BINS_AR_BK_B1_BIT_30       ,-1,  TO_AR(30), 0310, true);
   addParamToTable(P_BINS_AR_BK_B1_BIT_31       ,-1,  TO_AR(31), 0310, true);
   addParamToTable(P_BINS_AR_LK_B1              ,-1,  E_P_AR_H29_L9_S1_M90, 0311);
   addParamToTable(P_BINS_AR_LK_B1_BIT_30       ,-1,  TO_AR(30), 0311, true);
   addParamToTable(P_BINS_AR_LK_B1_BIT_31       ,-1,  TO_AR(31), 0311, true);
   addParamToTable(P_BINS_AR_WK_B1              ,-1,  E_P_AR_H29_L14_S1_M842_865, 0312);
   addParamToTable(P_BINS_AR_WK_B1_BIT_30       ,-1,  TO_AR(30), 0312, true);
   addParamToTable(P_BINS_AR_WK_B1_BIT_31       ,-1,  TO_AR(31), 0312, true);
   addParamToTable(P_BINS_AR_PU_B1              ,-1,  E_P_AR_H29_L14_S1_M90, 0313);
   addParamToTable(P_BINS_AR_PU_B1_BIT_30       ,-1,  TO_AR(30), 0313, true);
   addParamToTable(P_BINS_AR_PU_B1_BIT_31       ,-1,  TO_AR(31), 0313, true);
   addParamToTable(P_BINS_AR_OMEGA_PU_B1        ,-1,  E_P_AR_H29_L14_S1_M256, 0335);
   addParamToTable(P_BINS_AR_OMEGA_PU_B1_BIT_30 ,-1,  TO_AR(30), 0335, true);
   addParamToTable(P_BINS_AR_OMEGA_PU_B1_BIT_31 ,-1,  TO_AR(31), 0335, true);
   addParamToTable(P_BINS_AR_HDG_I_B1           ,-1,  E_P_AR_H29_L14_S1_M90, 0314);
   addParamToTable(P_BINS_AR_HDG_I_BIT_30       ,-1,  TO_AR(30), 0314, true);
   addParamToTable(P_BINS_AR_HDG_I_BIT_31       ,-1,  TO_AR(31), 0314, true);
   addParamToTable(P_BINS_AR_PUM_I_B1           ,-1,  E_P_AR_H29_L14_S1_M90, 0317);
   addParamToTable(P_BINS_AR_PUM_I_BIT_30       ,-1,  TO_AR(30), 0317, true);
   addParamToTable(P_BINS_AR_PUM_I_BIT_31       ,-1,  TO_AR(31), 0317, true);
   addParamToTable(P_BINS_AR_HDGM_I_B1          ,-1,  E_P_AR_H29_L14_S1_M90, 0320);
   addParamToTable(P_BINS_AR_HDGM_I_BIT_30      ,-1,  TO_AR(30), 0320, true);
   addParamToTable(P_BINS_AR_HDGM_I_BIT_31      ,-1,  TO_AR(31), 0320, true);

   addParamToTable(P_BINS_AR_PITCH_I_B1         ,-1,   E_P_AR_H29_L14_S1_M90, 0324);
   addParamToTable(P_BINS_AR_PITCH_I_BIT_30     ,-1,   TO_AR(30), 0324, true);
   addParamToTable(P_BINS_AR_PITCH_I_BIT_31     ,-1,   TO_AR(31), 0324, true);
   addParamToTable(P_BINS_AR_ROLL_I_B1          ,-1,   E_P_AR_H29_L14_S1_M90, 0325);
   addParamToTable(P_BINS_AR_ROLL_I_BIT_30      ,-1,   TO_AR(30), 0325, true);
   addParamToTable(P_BINS_AR_ROLL_I_BIT_31      ,-1,   TO_AR(31), 0325, true);
   addParamToTable(P_BINS_AR_HDG_GIR_I_B1       ,-1,   E_P_AR_H29_L14_S1_M90, 0334);
   addParamToTable(P_BINS_AR_HDG_GIR_I_BIT_30   ,-1,   TO_AR(30), 0334, true);
   addParamToTable(P_BINS_AR_HDG_GIR_I_BIT_31   ,-1,   TO_AR(31), 0334, true);

   addParamToTable(P_BINS_AR_VNK_B1       ,-1,   E_P_AR_H29_L14_S1_M842_865, 0366);
   addParamToTable(P_BINS_AR_VNK_BIT_30   ,-1,   TO_AR(30), 0366, true);
   addParamToTable(P_BINS_AR_VNK_BIT_31   ,-1,   TO_AR(31), 0366, true);

   addParamToTable(P_BINS_AR_VEK_B1       ,-1,   E_P_AR_H29_L14_S1_M842_865, 0367);
   addParamToTable(P_BINS_AR_VEK_BIT_30   ,-1,   TO_AR(30), 0367, true);
   addParamToTable(P_BINS_AR_VEK_BIT_31   ,-1,   TO_AR(31), 0367, true);

   addParamToTable(P_BINS_AR_VHK_B1       ,-1,   E_P_AR_H29_L14_S1_M842_865, 0365);
   addParamToTable(P_BINS_AR_VHK_BIT_30   ,-1,   TO_AR(30), 0365, true);
   addParamToTable(P_BINS_AR_VHK_BIT_31   ,-1,   TO_AR(31), 0365, true);

   addParamToTable(P_BINS_AR_OMEGA_X_I_B1       ,-1,   E_P_AR_H29_L14_S1_M256, 0327);
   addParamToTable(P_BINS_AR_OMEGA_X_I_BIT_30   ,-1,   TO_AR(30), 0327, true);
   addParamToTable(P_BINS_AR_OMEGA_X_I_BIT_31   ,-1,   TO_AR(31), 0327, true);

   addParamToTable(P_BINS_AR_OMEGA_Y_I_B1       ,-1,   E_P_AR_H29_L14_S1_M256, 0330);
   addParamToTable(P_BINS_AR_OMEGA_Y_I_BIT_30   ,-1,   TO_AR(30), 0330, true);
   addParamToTable(P_BINS_AR_OMEGA_Y_I_BIT_31   ,-1,   TO_AR(31), 0330, true);

   addParamToTable(P_BINS_AR_OMEGA_Z_I_B1       ,-1,   E_P_AR_H29_L14_S1_M256, 0326);
   addParamToTable(P_BINS_AR_OMEGA_Z_I_BIT_30   ,-1,   TO_AR(30), 0326, true);
   addParamToTable(P_BINS_AR_OMEGA_Z_I_BIT_31   ,-1,   TO_AR(31), 0326, true);

   addParamToTable(P_BINS_AR_AX_B1       ,-1,   E_P_AR_H29_L14_S1_M64, 0331);
   addParamToTable(P_BINS_AR_AX_BIT_30   ,-1,   TO_AR(30), 0331, true);
   addParamToTable(P_BINS_AR_AX_BIT_31   ,-1,   TO_AR(31), 0331, true);

   addParamToTable(P_BINS_AR_AY_B1       ,-1,   E_P_AR_H29_L14_S1_M64, 0333);
   addParamToTable(P_BINS_AR_AY_BIT_30   ,-1,   TO_AR(30), 0333, true);
   addParamToTable(P_BINS_AR_AY_BIT_31   ,-1,   TO_AR(31), 0333, true);

   addParamToTable(P_BINS_AR_AZ_B1       ,-1,   E_P_AR_H29_L14_S1_M64, 0332);
   addParamToTable(P_BINS_AR_AZ_BIT_30   ,-1,   TO_AR(30), 0332, true);
   addParamToTable(P_BINS_AR_AZ_BIT_31   ,-1,   TO_AR(31), 0332, true);

   addParamToTable(P_BINS_AR_AG_B1       ,-1,   E_P_AR_H29_L14_S1_M64, 0364);
   addParamToTable(P_BINS_AR_AG_BIT_30   ,-1,   TO_AR(30), 0364, true);
   addParamToTable(P_BINS_AR_AG_BIT_31   ,-1,   TO_AR(31), 0364, true);

   addParamToTable(P_BINS_AR_UG_PLF_B1       ,-1,   E_P_AR_H29_L14_S1_M90, 0226);
   addParamToTable(P_BINS_AR_UG_PLF_BIT_30   ,-1,   TO_AR(30), 0226, true);
   addParamToTable(P_BINS_AR_UG_PLF_BIT_31   ,-1,   TO_AR(31), 0226, true);

   addParamToTable(P_BINS_AR_VY_POT_I_B1        ,-1,   E_P_AR_H29_L14_S1_M842_865, 0360);
   addParamToTable(P_BINS_AR_VY_POT_I_BIT_30    ,-1,   TO_AR(30), 0360, true);
   addParamToTable(P_BINS_AR_VY_POT_I_BIT_31    ,-1,   TO_AR(31), 0360, true);
   addParamToTable(P_BINS_AR_HBI_I_B1           ,-1,   E_P_AR_H29_L14_S1_M16384, 0361);
   addParamToTable(P_BINS_AR_HBI_I_BIT_30       ,-1,   TO_AR(30), 0361, true);
   addParamToTable(P_BINS_AR_HBI_I_BIT_31       ,-1,   TO_AR(31), 0361, true);
   addParamToTable(P_BINS_AR_U_B1               ,-1,   E_P_AR_H29_L14_S1_M842_865, 0315);
   addParamToTable(P_BINS_AR_U_BIT_30           ,-1,   TO_AR(30), 0315, true);
   addParamToTable(P_BINS_AR_U_BIT_31           ,-1,   TO_AR(31), 0315, true);
   addParamToTable(P_BINS_AR_UG_V_B1            ,-1,   E_P_AR_H29_L14_S1_M90, 0316);
   addParamToTable(P_BINS_AR_UG_V_BIT_30        ,-1,   TO_AR(30), 0316, true);
   addParamToTable(P_BINS_AR_UG_V_BIT_31        ,-1,   TO_AR(31), 0316, true);
   addParamToTable(P_BINS_AR_US_B1              ,-1,   E_P_AR_H29_L14_S1_M90, 0321);
   addParamToTable(P_BINS_AR_US_BIT_30          ,-1,   TO_AR(30), 0321, true);
   addParamToTable(P_BINS_AR_US_BIT_31          ,-1,   TO_AR(31), 0321, true);
   addParamToTable(P_BINS_AR_UG_TR_B1           ,-1,   E_P_AR_H29_L14_S1_M90, 0322);
   addParamToTable(P_BINS_AR_UG_TR_BIT_30       ,-1,   TO_AR(30), 0322, true);
   addParamToTable(P_BINS_AR_UG_TR_BIT_31       ,-1,   TO_AR(31), 0322, true);

   //LL
   addParamToTable(P_BINS_TZAD1_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 1);
   addParamToTable(P_BINS_TZAD1_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 1);


   addParamToTable(P_BINS_SS1_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 2);
   addParamToTable(P_BINS_SS1_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S0_M0, 2);


   addParamToTable(P_BINS_ROLL_I_B1,     MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 10);
   addParamToTable(P_BINS_ROLL_I_B1,     MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 10);

   addParamToTable(P_BINS_PITCH_I_B1,     MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 11);
   addParamToTable(P_BINS_PITCH_I_B1,     MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 11);

   addParamToTable(P_BINS_HDG_I_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 12);
   addParamToTable(P_BINS_HDG_I_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 12);


   addParamToTable(P_BINS_AX_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 4);
   addParamToTable(P_BINS_AX_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 4);


   addParamToTable(P_BINS_AZ_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 5);
   addParamToTable(P_BINS_AZ_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 5);


   addParamToTable(P_BINS_AY_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 6);
   addParamToTable(P_BINS_AY_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M64, 6);


   addParamToTable(P_BINS_OMEGA_X_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 7);
   addParamToTable(P_BINS_OMEGA_X_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 7);


   addParamToTable(P_BINS_OMEGA_Z_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 8);
   addParamToTable(P_BINS_OMEGA_Z_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 8);


   addParamToTable(P_BINS_OMEGA_Y_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 9);
   addParamToTable(P_BINS_OMEGA_Y_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M256, 9);



   addParamToTable(P_BINS_VNI_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 13);
   addParamToTable(P_BINS_VNI_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 13);


   addParamToTable(P_BINS_VEI_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 15);
   addParamToTable(P_BINS_VEI_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 15);


   addParamToTable(P_BINS_VHBI_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 17);
   addParamToTable(P_BINS_VHBI_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M842_8658, 17);


   addParamToTable(P_BINS_BI_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 19);
   addParamToTable(P_BINS_BI_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 19);


   addParamToTable(P_BINS_LI_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 21);
   addParamToTable(P_BINS_LI_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L35_S1_M180, 21);


   addParamToTable(P_BINS_HBI_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M16384, 23);
   addParamToTable(P_BINS_HBI_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M16384, 23);


   addParamToTable(P_BINS_HDGM_I_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 24);
   addParamToTable(P_BINS_HDGM_I_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_M180, 24);


   addParamToTable(P_BINS_DELTA_M_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S1_MMPI, 25);
   addParamToTable(P_BINS_DELTA_M_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S1_MMPI, 25);


   addParamToTable(P_BINS_T_WORK_B1,       MIL_OUT_BINS1_58_SP2_SA1,   E_P_MIL_H4_L19_S0_M32760, 26);
   addParamToTable(P_BINS_T_WORK_B1,       MIL_OUT_BINS2_58_SP2_SA1,   E_P_MIL_H4_L19_S0_M32760, 26);

   addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 1);
     addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 1);

   addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 1);
   addParamToTable(P_BINS_TZAD1_B1_MC1D3,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 1);

   addParamToTable(P_BINS_T_ZAD2_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 25);
   addParamToTable(P_BINS_T_ZAD2_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 25);

   addParamToTable(P_BINS_SD2_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 2);
   addParamToTable(P_BINS_SD2_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 2);

   addParamToTable(P_BINS_VNSNS1_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 3);
   addParamToTable(P_BINS_VNSNS1_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 3);


   addParamToTable(P_BINS_VESNS1_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 5);
   addParamToTable(P_BINS_VESNS1_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 5);


   addParamToTable(P_BINS_VHSNS1_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 7);
   addParamToTable(P_BINS_VHSNS1_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M842_8658, 7);


   addParamToTable(P_BINS_BSNS_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 9);
   addParamToTable(P_BINS_BSNS_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 9);


   addParamToTable(P_BINS_LSNS_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 11);
   addParamToTable(P_BINS_LSNS_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L35_S1_M180, 11);


   addParamToTable(P_BINS_HSNS_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 13);
   addParamToTable(P_BINS_HSNS_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 13);

   //!!
   addParamToTable(P_BINS_TSNS_H_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 14);
   addParamToTable(P_BINS_TSNS_H_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L19_S1_M16384, 14);


   addParamToTable(P_BINS_TSNS_M_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H9_L14_S0_M0, 14);
   addParamToTable(P_BINS_TSNS_M_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H9_L14_S0_M0, 14);


   addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);
   addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);


   addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);
   addParamToTable(P_BINS_TSNS_S_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L9_S0_M0, 15);


   addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);
   addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);


   addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);
   addParamToTable(P_BINS_TS_EXACT_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L23_S0_M0_5, 15);


   addParamToTable(P_BINS_DATSNS,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 18);
   addParamToTable(P_BINS_DATSNS,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 18);


   addParamToTable(P_BINS_GF_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M3276_8, 19);
   addParamToTable(P_BINS_GF_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M3276_8, 19);


   addParamToTable(P_BINS_NLGN_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);
   addParamToTable(P_BINS_NLGN_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);


   addParamToTable(P_BINS_NGPS_B1,       MIL_OUT_BINS1_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);
   addParamToTable(P_BINS_NGPS_B1,       MIL_OUT_BINS2_58_SP2_SA2,   E_P_MIL_H4_L19_S0_M0, 20);



   addParamToTable(P_BINS_SS1_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 2);
   addParamToTable(P_BINS_SS1_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L19_S0_M0, 2);


   addParamToTable(P_BINS_ROLL_K_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 4);
   addParamToTable(P_BINS_ROLL_K_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 4);


   addParamToTable(P_BINS_PITCH_K_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 5);
   addParamToTable(P_BINS_PITCH_K_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 5);


   addParamToTable(P_BINS_HDG_K_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 6);
   addParamToTable(P_BINS_HDG_K_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 6);


   addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);
   addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);


   addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);
   addParamToTable(P_BINS_HDGM_K_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M180, 7);


   addParamToTable(P_BINS_VNK_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 8);
   addParamToTable(P_BINS_VNK_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 8);


   addParamToTable(P_BINS_VEK_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 10);
   addParamToTable(P_BINS_VEK_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 10);



   addParamToTable(P_BINS_VHK_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 12);
   addParamToTable(P_BINS_VHK_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M842_8658, 12);


   addParamToTable(P_BINS_BK_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 13);
   addParamToTable(P_BINS_BK_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 13);



   addParamToTable(P_BINS_LK_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 15);
   addParamToTable(P_BINS_LK_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L35_S1_M180, 15);


   addParamToTable(P_BINS_HK_B1_MC1D1,       MIL_OUT_BINS1_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M16384, 17);
   addParamToTable(P_BINS_HK_B1_MC1D1,       MIL_OUT_BINS2_58_SP2_SA3,   E_P_MIL_H4_L19_S1_M16384, 17);


   addParamToTable(P_BINS_SS1_B1_MC1D3,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 2);
   addParamToTable(P_BINS_SS1_B1_MC1D3,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L19_S0_M0, 2);


   addParamToTable(P_BINS_ROLL_I_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 4);
   addParamToTable(P_BINS_ROLL_I_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 4);


   addParamToTable(P_BINS_PITCH_I_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 5);
   addParamToTable(P_BINS_PITCH_I_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 5);


   addParamToTable(P_BINS_HDG_I_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 6);
   addParamToTable(P_BINS_HDG_I_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M180, 6);


   addParamToTable(P_BINS_VNI_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 7);
   addParamToTable(P_BINS_VNI_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 7);


   addParamToTable(P_BINS_VEI_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 9);
   addParamToTable(P_BINS_VEI_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 9);


   addParamToTable(P_BINS_VHBI_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 11);
   addParamToTable(P_BINS_VHBI_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L35_S1_M842_8658, 11);


   addParamToTable(P_BINS_OMEGA_X_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 13);
   addParamToTable(P_BINS_OMEGA_X_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 13);


   addParamToTable(P_BINS_OMEGA_Z_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 14);
   addParamToTable(P_BINS_OMEGA_Z_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 14);


   addParamToTable(P_BINS_OMEGA_Y_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 15);
   addParamToTable(P_BINS_OMEGA_Y_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA4,   E_P_MIL_H4_L19_S1_M256, 15);




   /////////////////////////////////////////////////


   addParamToTable(P_BINS_ROLL_K_B1_MC1D2,   MIL_OUT_BINS1_58_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 16);
   addParamToTable(P_BINS_ROLL_K_B1_MC1D2,   MIL_OUT_BINS2_58_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 16);

   addParamToTable(P_BINS_PITCH_K_B1_MC1D2,  MIL_OUT_BINS1_58_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 17);
   addParamToTable(P_BINS_PITCH_K_B1_MC1D2,  MIL_OUT_BINS2_58_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 17);

   addParamToTable(P_BINS_HDG_K_B1_MC1D2,    MIL_OUT_BINS1_58_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 18);
   addParamToTable(P_BINS_HDG_K_B1_MC1D2,    MIL_OUT_BINS2_58_SP2_SA4, E_P_MIL_H4_L19_S1_M180, 18);

   addParamToTable(P_BINS_VNK_B1_MC1D2,      MIL_OUT_BINS1_58_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 19);
   addParamToTable(P_BINS_VNK_B1_MC1D2,      MIL_OUT_BINS2_58_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 19);

   addParamToTable(P_BINS_VEK_B1_MC1D2,      MIL_OUT_BINS1_58_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 21);
   addParamToTable(P_BINS_VEK_B1_MC1D2,      MIL_OUT_BINS2_58_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 21);

   addParamToTable(P_BINS_VHBIK_B1_MC1D2,    MIL_OUT_BINS1_58_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 23);
   addParamToTable(P_BINS_VHBIK_B1_MC1D2,    MIL_OUT_BINS2_58_SP2_SA4, E_P_MIL_H4_L35_S1_M842_8658, 23);

   addParamToTable(P_BINS_BK_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 26);
   addParamToTable(P_BINS_BK_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 26);

   addParamToTable(P_BINS_LK_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 28);
   addParamToTable(P_BINS_LK_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA3, E_P_MIL_H4_L35_S1_M180, 28);

   addParamToTable(P_BINS_HK_B1_MC1D2,       MIL_OUT_BINS1_58_SP2_SA3, E_P_MIL_H4_L19_S1_M16384, 30);
   addParamToTable(P_BINS_HK_B1_MC1D2,       MIL_OUT_BINS2_58_SP2_SA3, E_P_MIL_H4_L19_S1_M16384, 30);

   addParamToTable(P_BINS_SKO_B1_MC1,        MIL_OUT_BINS1_58_SP2_SA3, E_P_MIL_H4_L19_S0_M0, 31);
   addParamToTable(P_BINS_SKO_B1_MC1,        MIL_OUT_BINS2_58_SP2_SA3, E_P_MIL_H4_L19_S0_M0, 31);

   addParamToTable(P_BINS_SS1_B1_MC1D4,      MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S0_M0, 2);
   addParamToTable(P_BINS_SS1_B1_MC1D4,      MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S0_M0, 2);

   addParamToTable(P_BINS_WK_B1_MC1,         MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 3);
   addParamToTable(P_BINS_WK_B1_MC1,         MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 3);

   addParamToTable(P_BINS_PU_B1_MC1,         MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 4);
   addParamToTable(P_BINS_PU_B1_MC1,         MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 4);

   addParamToTable(P_BINS_PUM_B1_MC1,        MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 5);
   addParamToTable(P_BINS_PUM_B1_MC1,        MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 5);

   addParamToTable(P_BINS_ATR_B1_MC1,        MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 6);
   addParamToTable(P_BINS_ATR_B1_MC1,        MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 6);

   addParamToTable(P_BINS_AG_B1_MC1,         MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 7);
   addParamToTable(P_BINS_AG_B1_MC1,         MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 7);

   addParamToTable(P_BINS_UG_TR_B1_MC1,      MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 8);
   addParamToTable(P_BINS_UG_TR_B1_MC1,      MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_M64, 8);

   addParamToTable(P_BINS_VY_POT_B1_MC1,     MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_M842_865, 9);
   addParamToTable(P_BINS_VY_POT_B1_MC1,     MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_M842_865, 9);

   addParamToTable(P_BINS_U_B1_MC1,          MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 10);
   addParamToTable(P_BINS_U_B1_MC1,          MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S0_M842_865, 10);

   addParamToTable(P_BINS_UG_V_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 11);
   addParamToTable(P_BINS_UG_V_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 11);

   addParamToTable(P_BINS_OMEGA_PU_B1_MC1,   MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_M256, 12);
   addParamToTable(P_BINS_OMEGA_PU_B1_MC1,   MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_M256, 12);

   addParamToTable(P_BINS_US_B1_MC1,         MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 13);
   addParamToTable(P_BINS_US_B1_MC1,         MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 13);

   addParamToTable(P_BINS_UG_PLF_B1_MC1,     MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 14);
   addParamToTable(P_BINS_UG_PLF_B1_MC1,     MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 14);

   addParamToTable(P_BINS_NPMO_B1_MC1,       MIL_OUT_BINS1_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 15);
   addParamToTable(P_BINS_NPMO_B1_MC1,       MIL_OUT_BINS2_58_SP2_SA5, E_P_MIL_H4_L19_S1_MMPI, 15);

   addParamToTable(P_BINS_DOST_USKOR_B1,     MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(4), 3,true);
   addParamToTable(P_BINS_DOST_USKOR_B1,     MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(4), 3,true);

   addParamToTable(P_BINS_DOST_OMEGA_B1,     MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(5), 3,true);
   addParamToTable(P_BINS_DOST_OMEGA_B1,     MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(5), 3,true);

   addParamToTable(P_BINS_DOST_VERTIK_I_B1,  MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(6), 3,true);
   addParamToTable(P_BINS_DOST_VERTIK_I_B1,  MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(6), 3,true);

   addParamToTable(P_BINS_DOST_HDG_I_B1,     MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(7), 3,true);
   addParamToTable(P_BINS_DOST_HDG_I_B1,     MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(7), 3,true);

   addParamToTable(P_BINS_DOST_HDGM_I_B1,    MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(8), 3,true);
   addParamToTable(P_BINS_DOST_HDGM_I_B1,    MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(8), 3,true);

   addParamToTable(P_BINS_DOST_DELTA_M_B1,   MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(9), 3,true);
   addParamToTable(P_BINS_DOST_DELTA_M_B1,   MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(9), 3,true);

   addParamToTable(P_BINS_DOST_SKOR_I_B1,    MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(10), 3,true);
   addParamToTable(P_BINS_DOST_SKOR_I_B1,    MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(10), 3,true);

   addParamToTable(P_BINS_DOST_KOORD_I_B1,   MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(11), 3,true);
   addParamToTable(P_BINS_DOST_KOORD_I_B1,   MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(11), 3,true);

   addParamToTable(P_BINS_DOST_HBI_B1,       MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(12), 3,true);
   addParamToTable(P_BINS_DOST_HBI_B1,       MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(12), 3,true);

   addParamToTable(P_BINS_DOST_TR_B1,        MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(13), 3,true);
   addParamToTable(P_BINS_DOST_TR_B1,        MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(13), 3,true);

   addParamToTable(P_BINS_DOST_H_BAR_B1,        MIL_OUT_BINS1_58_SP2_SA1, TO_MIL(15), 3,true);
   addParamToTable(P_BINS_DOST_H_BAR_B1,        MIL_OUT_BINS2_58_SP2_SA1, TO_MIL(15), 3,true);




   addParamToTable(P_BINS_CY1		, MIL_IN_BINS1_58_SP2_SA1, E_P_MIL_H4_L19_S0_M0, 1);
   addParamToTable(P_BINS_CY1		, MIL_IN_BINS2_58_SP2_SA1, E_P_MIL_H4_L19_S0_M0, 1);

   addParamToTable(P_BINS_CY2		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 1);
   addParamToTable(P_BINS_CY2		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 1);

   addParamToTable(P_BINS_CD2		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 2);
   addParamToTable(P_BINS_CD2		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 2);

   addParamToTable(P_BINS_FI_IN		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 3);
   addParamToTable(P_BINS_FI_IN		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 3);

   addParamToTable(P_BINS_LAMBDA_IN		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 5);
   addParamToTable(P_BINS_LAMBDA_IN		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 5);

   addParamToTable(P_BINS_HOURB		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L8_S0_M0, 7);
   addParamToTable(P_BINS_HOURB		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L8_S0_M0, 7);

   addParamToTable(P_BINS_MINB		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L14_S0_M0, 7);
   addParamToTable(P_BINS_MINB		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L14_S0_M0, 7);

   addParamToTable(P_BINS_SECB		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L9_S0_M0, 8);
   addParamToTable(P_BINS_SECB		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L9_S0_M0, 8);

   addParamToTable(P_BINS_DATB		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 9);
   addParamToTable(P_BINS_DATB		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 9);

   addParamToTable(P_BINS_ALGWORDS_0		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 10);
   addParamToTable(P_BINS_ALGWORDS_0		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 10);

   addParamToTable(P_BINS_ALGWORDS_1		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 11);
   addParamToTable(P_BINS_ALGWORDS_1		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 11);

   addParamToTable(P_BINS_ALGWORDS_2		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 12);
   addParamToTable(P_BINS_ALGWORDS_2		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 12);

   addParamToTable(P_BINS_ALGWORDS_3		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 13);
   addParamToTable(P_BINS_ALGWORDS_3		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 13);

   addParamToTable(P_BINS_ALGWORDS_4		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 14);
   addParamToTable(P_BINS_ALGWORDS_4		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 14);

   addParamToTable(P_BINS_ALGWORDS_5		, MIL_IN_BINS1_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 15);
   addParamToTable(P_BINS_ALGWORDS_5		, MIL_IN_BINS2_58_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 15);


   ////////////////////////////////////

    addParamToTable(P_KSU_H_ABS		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M16384, 1);
    addParamToTable(P_KSU_H_ABS		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M16384, 1);

    addParamToTable(P_KSU_H_ABS		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M16384, 1);
    addParamToTable(P_KSU_H_ABS		, AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M16384, 0203);
    addParamToTable(P_KSU_H_ABS_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0203);
    addParamToTable(P_KSU_H_ABS		, AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M16384, 0203);

    addParamToTable(P_KSU_H_ABS		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M16384 , 1);
    addParamToTable(P_KSU_H_ABS		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M16384 , 1);
    addParamToTable(P_KSU_H_ABS		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M16384 , 1);
    addParamToTable(P_KSU_H_ABS		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M16384 , 1);


    addParamToTable(P_KSU_V_PR		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M819_2, 2);
    addParamToTable(P_KSU_V_PR		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M819_2, 2);
    addParamToTable(P_KSU_V_PR		, AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M819_2, 0206);
    addParamToTable(P_KSU_V_PR		, AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M819_2, 0206);

    addParamToTable(P_KSU_V_PR		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M819_2, 3);
    addParamToTable(P_KSU_V_PR		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M819_2, 3);
    addParamToTable(P_KSU_V_PR		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M819_2, 3);
    addParamToTable(P_KSU_V_PR		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M819_2, 3);

    addParamToTable(P_KSU_V_PR_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0206);

    addParamToTable(P_KSU_V_IST		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M2048, 3);
    addParamToTable(P_KSU_V_IST		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M2048, 3);
    addParamToTable(P_KSU_V_IST		, AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M2048, 0210);
    addParamToTable(P_KSU_V_IST		, AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M2048, 0210);

    addParamToTable(P_KSU_V_IST_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0210);

    addParamToTable(P_KSU_V_IST		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M2048, 4);
    addParamToTable(P_KSU_V_IST		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M2048, 4);
    addParamToTable(P_KSU_V_IST		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M2048, 4);
    addParamToTable(P_KSU_V_IST		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M2048, 4);


    addParamToTable(P_KSU_M		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M2_048, 4);
    addParamToTable(P_KSU_M		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M2_048, 4);
    addParamToTable(P_KSU_M		, AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M2_048, 0205);
    addParamToTable(P_KSU_M		, AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M2_048, 0205);

    addParamToTable(P_KSU_M		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M2_048, 5);
    addParamToTable(P_KSU_M		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M2_048, 5);
    addParamToTable(P_KSU_M		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M2_048, 5);
    addParamToTable(P_KSU_M		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M2_048, 5);

    addParamToTable(P_KSU_M_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0205);



    addParamToTable(P_KSU_VY		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M256, 5);
    addParamToTable(P_KSU_VY		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M256, 5);
    addParamToTable(P_KSU_VY		, AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M256, 0212);
    addParamToTable(P_KSU_VY		, AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M256, 0212);

    addParamToTable(P_KSU_VY		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M256, 6);
    addParamToTable(P_KSU_VY		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M256, 6);
    addParamToTable(P_KSU_VY		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M256, 6);
    addParamToTable(P_KSU_VY		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M256, 6);

    addParamToTable(P_KSU_VY_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0212);

    addParamToTable(P_KSU_H_OTN		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M16384, 6);
    addParamToTable(P_KSU_H_OTN		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M16384, 6);
    addParamToTable(P_KSU_H_OTN		, AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M16384, 0204);
    addParamToTable(P_KSU_H_OTN		, AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M16384, 0204);

    addParamToTable(P_KSU_H_OTN		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M16384 , 2);
    addParamToTable(P_KSU_H_OTN		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M16384 , 2);
    addParamToTable(P_KSU_H_OTN		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M16384 , 2);
    addParamToTable(P_KSU_H_OTN		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M16384 , 2);

    addParamToTable(P_KSU_H_OTN_MAT	, AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0, 0204);

    //addParamToTable(P_KSU_H_OTN		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M16384, 6);
    addParamToTable(P_KSU_PABS		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_KSU_PABS		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_KSU_PDIF		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M64, 8);
    addParamToTable(P_KSU_PDIF		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M64, 8);

    addParamToTable(P_KSU_PABS		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_KSU_PABS		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_KSU_PABS		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M64, 7);
    addParamToTable(P_KSU_PABS		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M64, 7);

    addParamToTable(P_KSU_PDIF		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M64, 8);
    addParamToTable(P_KSU_PDIF		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M64, 8);
    addParamToTable(P_KSU_PDIF		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M64, 8);
    addParamToTable(P_KSU_PDIF		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M64, 8);

    addParamToTable(P_KSU_P_Z		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M512, 9);
    addParamToTable(P_KSU_P_Z		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M512, 9);
    addParamToTable(P_KSU_P_Z		, AR_IN_KSU_UCOKS1, E_P_AR_H28_L14_S0_M512, 034);
    addParamToTable(P_KSU_P_Z		, AR_OUT_KSU_L_UCOKS1, TO_MIL(11), 0250,true);

    addParamToTable(P_KSU_P_Z		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M512, 10);
    addParamToTable(P_KSU_P_Z		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M512, 10);
    addParamToTable(P_KSU_P_Z		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M512, 10);
    addParamToTable(P_KSU_P_Z		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M512, 10);

    addParamToTable(P_KSU_TN		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M512, 13);
    addParamToTable(P_KSU_TN		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M512, 13);
    addParamToTable(P_KSU_TN		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M512, 13);
    addParamToTable(P_KSU_TN		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M512, 13);

    addParamToTable(P_KSU_TT		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M512, 14);
    addParamToTable(P_KSU_TT		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M512, 14);
    addParamToTable(P_KSU_TT		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M512, 14);
    addParamToTable(P_KSU_TT		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M512, 14);

    addParamToTable(P_KSU_ALFA_IST		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M90, 10);
    addParamToTable(P_KSU_ALFA_IST		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M90, 10);
    addParamToTable(P_KSU_ALFA_IST         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 0221);
    addParamToTable(P_KSU_ALFA_IST         , AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M90, 0221);

    addParamToTable(P_KSU_ALFA_IST		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90, 11);
    addParamToTable(P_KSU_ALFA_IST		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90, 11);
    addParamToTable(P_KSU_ALFA_IST		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90, 11);
    addParamToTable(P_KSU_ALFA_IST		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90, 11);

    addParamToTable(P_KSU_ALFA_IST_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0221);

    addParamToTable(P_KSU_BETA_IST		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M90, 11);
    addParamToTable(P_KSU_BETA_IST		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M90, 11);
    addParamToTable(P_KSU_BETA_IST              , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 0321);
    addParamToTable(P_KSU_BETA_IST              , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 0321);

    addParamToTable(P_KSU_BETA_IST		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S1_M90, 12);
    addParamToTable(P_KSU_BETA_IST		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S1_M90, 12);
    addParamToTable(P_KSU_BETA_IST		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S1_M90, 12);
    addParamToTable(P_KSU_BETA_IST		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S1_M90, 12);
    addParamToTable(P_KSU_BETA_IST_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0321);

    addParamToTable(P_KSU_ALFA_MAX		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M90, 12);
    addParamToTable(P_KSU_ALFA_MAX		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M90, 12);
    addParamToTable(P_KSU_ALFA_MAX              , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 0240);

    addParamToTable(P_KSU_ALFA_MAX		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90, 15);
    addParamToTable(P_KSU_ALFA_MAX		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90, 15);
    addParamToTable(P_KSU_ALFA_MAX		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90, 15);
    addParamToTable(P_KSU_ALFA_MAX		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90, 15);

    addParamToTable(P_KSU_ALFA_MAX_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0240);

    addParamToTable(P_KSU_ALFA_MIN		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M90, 13);
    addParamToTable(P_KSU_ALFA_MIN		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M90, 13);
    addParamToTable(P_KSU_ALFA_MIN              , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 061);

    addParamToTable(P_KSU_ALFA_MIN		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90 , 16);
    addParamToTable(P_KSU_ALFA_MIN		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90 , 16);
    addParamToTable(P_KSU_ALFA_MIN		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90 , 16);
    addParamToTable(P_KSU_ALFA_MIN		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90 , 16);
    addParamToTable(P_KSU_ALFA_MIN_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 061);

    addParamToTable(P_KSU_BETA_DOP		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M90, 14);
    addParamToTable(P_KSU_BETA_DOP		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M90, 14);
    addParamToTable(P_KSU_BETA_DOP              , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 0302);

    addParamToTable(P_KSU_BETA_DOP		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90 , 22);
    addParamToTable(P_KSU_BETA_DOP		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90 , 22);
    addParamToTable(P_KSU_BETA_DOP		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90 , 22);
    addParamToTable(P_KSU_BETA_DOP		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90 , 22);
    addParamToTable(P_KSU_BETA_DOP_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0302);

    addParamToTable(P_KSU_NY_MAX		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M8, 15);
    addParamToTable(P_KSU_NY_MAX		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M8, 15);
    addParamToTable(P_KSU_NY_MAX                , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 0113);

    addParamToTable(P_KSU_NY_MAX		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90 , 17);
    addParamToTable(P_KSU_NY_MAX		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90 , 17);
    addParamToTable(P_KSU_NY_MAX		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90 , 17);
    addParamToTable(P_KSU_NY_MAX		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90 , 17);
    addParamToTable(P_KSU_NY_MAX_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0113);

    addParamToTable(P_KSU_NY_MIN		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90 , 18);
    addParamToTable(P_KSU_NY_MIN		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90 , 18);
    addParamToTable(P_KSU_NY_MIN		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90 , 18);
    addParamToTable(P_KSU_NY_MIN		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90 , 18);



    addParamToTable(P_KSU_NY_MIN		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M4, 16);
    addParamToTable(P_KSU_NY_MIN		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M4, 16);
    addParamToTable(P_KSU_NY_MIN                , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M90, 0300);

    addParamToTable(P_KSU_NY_MIN_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0300);
    addParamToTable(P_KSU_V_MAX		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M812, 17);
    addParamToTable(P_KSU_V_MAX		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M812, 17);
    addParamToTable(P_KSU_V_MAX                , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M819_2, 073);

    addParamToTable(P_KSU_V_MAX		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90 , 19);
    addParamToTable(P_KSU_V_MAX		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90 , 19);
    addParamToTable(P_KSU_V_MAX		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90 , 19);
    addParamToTable(P_KSU_V_MAX		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90 , 19);
    addParamToTable(P_KSU_V_MAX_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 073);


    addParamToTable(P_KSU_V_MIN		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M812, 18);
    addParamToTable(P_KSU_V_MIN		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M812, 18);
    addParamToTable(P_KSU_V_MIN                , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M819_2, 0245);

    addParamToTable(P_KSU_V_MIN		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90 , 20);
    addParamToTable(P_KSU_V_MIN		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90 , 20);
    addParamToTable(P_KSU_V_MIN		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90 , 20);
    addParamToTable(P_KSU_V_MIN		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90 , 20);
    addParamToTable(P_KSU_V_MIN_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0245);


    addParamToTable(P_KSU_M_DOP		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S1_M2_048, 19);
    addParamToTable(P_KSU_M_DOP		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S1_M2_048, 19);
    addParamToTable(P_KSU_M_DOP         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M2_048, 0301);

    addParamToTable(P_KSU_M_DOP		, MIL_OUT_KSU58_NVG1_SA1,  E_P_MIL_H4_L19_S1_M90 , 21);
    addParamToTable(P_KSU_M_DOP		, MIL_OUT_KSU58_NVG2_SA1,  E_P_MIL_H4_L19_S1_M90 , 21);
    addParamToTable(P_KSU_M_DOP		, MIL_OUT_KSU58_BP1_SA1,   E_P_MIL_H4_L19_S1_M90 , 21);
    addParamToTable(P_KSU_M_DOP		, MIL_OUT_KSU58_BP2_SA1,   E_P_MIL_H4_L19_S1_M90 , 21);
    addParamToTable(P_KSU_M_DOP_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 0301);


    addParamToTable(P_KSU_SRK1		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_KSU_SRK1		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_KSU_SRK1      , AR_IN_KSU_UCOKS1,    E_P_AR_H28_L1_S0_M0, 01);
    addParamToTable(P_KSU_SRK1      , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0, 01);

    addParamToTable(P_KSU_SRK1		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK1		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK1		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK1		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK1_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 01);


    addParamToTable(P_KSU_SRK2		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK2		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK2          , AR_IN_KSU_UCOKS1,    E_P_AR_H28_L1_S0_M0, 02);
    addParamToTable(P_KSU_SRK2      , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0, 02);

    addParamToTable(P_KSU_SRK2		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK2		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK2		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK2		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK2_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 02);



    addParamToTable(P_KSU_SRK15		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_KSU_SRK15		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_KSU_SRK15		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_KSU_SRK15		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 31);

    addParamToTable(P_KSU_SRK3		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S0_M0, 22);
    addParamToTable(P_KSU_SRK3		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S0_M0, 22);
    addParamToTable(P_KSU_SRK3          , AR_IN_KSU_UCOKS1,    E_P_AR_H28_L1_S0_M0, 03);
    addParamToTable(P_KSU_SRK3      , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0, 03);

    addParamToTable(P_KSU_SRK3		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK3		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK3		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK3		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 25);

    addParamToTable(P_KSU_SRK3_MAT     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H31_L30_S0_M0    , 03);

    addParamToTable(P_KSU_SRK14		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK14		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK14		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK14		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK14		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK14		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 30);

    addParamToTable(P_KSU_SRK14     , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0, 016);
    addParamToTable(P_KSU_SRK14_MAT     , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 016);


    addParamToTable(P_KSU_SRK51		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK51		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK52		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK52		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK52		, MIL_OUT_KSU_NVG_SA1, E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK52		, MIL_OUT_KSU_OSO_SA1, E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_N_Y		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M8, 1);
    addParamToTable(P_KSU_N_Y		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M8, 1);
    addParamToTable(P_KSU_N_Y         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M8, 0364);
    addParamToTable(P_KSU_N_Y         , AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M8, 0364);
    addParamToTable(P_KSU_N_Y		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M8, 4);
    addParamToTable(P_KSU_N_Y		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M8, 4);
    addParamToTable(P_KSU_N_Y		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M8, 4);
    addParamToTable(P_KSU_N_Y		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M8, 4);


    addParamToTable(P_KSU_N_Z		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M4, 2);
    addParamToTable(P_KSU_N_Z		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M4, 2);
    addParamToTable(P_KSU_N_Z         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M4, 0363);
    addParamToTable(P_KSU_N_Z         , AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M4, 0363);
    addParamToTable(P_KSU_N_Z		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M4, 6);
    addParamToTable(P_KSU_N_Z		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M4, 6);
    addParamToTable(P_KSU_N_Z		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M4, 6);
    addParamToTable(P_KSU_N_Z		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M4, 6);

    addParamToTable(P_KSU_N_X		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M4, 3);
    addParamToTable(P_KSU_N_X		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M4, 3);
    addParamToTable(P_KSU_N_X		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M4, 5);
    addParamToTable(P_KSU_N_X		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M4, 5);
    addParamToTable(P_KSU_N_X		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M4, 5);
    addParamToTable(P_KSU_N_X		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M4, 5);


    addParamToTable(P_KSU_DIR_PROD         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M1, 0141);
    addParamToTable(P_KSU_DIR_PROD         , AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M1, 0141);
    addParamToTable(P_KSU_DIR_PROD         , MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M0_5, 1);
    addParamToTable(P_KSU_DIR_PROD         , MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M0_5, 1);
    addParamToTable(P_KSU_DIR_PROD         , MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M0_5, 1);
    addParamToTable(P_KSU_DIR_PROD         , MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M0_5, 1);


    addParamToTable(P_KSU_DIR_KREN         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M1, 0140);
    addParamToTable(P_KSU_DIR_KREN         , AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M1, 0140);
    addParamToTable(P_KSU_DIR_KREN         , MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M0_5, 2);
    addParamToTable(P_KSU_DIR_KREN         , MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M0_5, 2);
    addParamToTable(P_KSU_DIR_KREN         , MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M0_5, 2);
    addParamToTable(P_KSU_DIR_KREN         , MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M0_5, 2);

    addParamToTable(P_KSU_V_ZAD_PRIB ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M1024, 3);
    addParamToTable(P_KSU_V_ZAD_PRIB ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M1024, 3);
    addParamToTable(P_KSU_V_ZAD_PRIB ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M1024, 3);
    addParamToTable(P_KSU_V_ZAD_PRIB ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M1024, 3);


    addParamToTable(P_KSU_OTKL_GAMMA_ZAD ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M64, 4);
    addParamToTable(P_KSU_OTKL_GAMMA_ZAD ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M64, 4);
    addParamToTable(P_KSU_OTKL_GAMMA_ZAD ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M64, 4);
    addParamToTable(P_KSU_OTKL_GAMMA_ZAD ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M64, 4);

    addParamToTable(P_KSU_OTKL_NY_ZAD ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M2, 5);
    addParamToTable(P_KSU_OTKL_NY_ZAD ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M2, 5);
    addParamToTable(P_KSU_OTKL_NY_ZAD ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M2, 5);
    addParamToTable(P_KSU_OTKL_NY_ZAD ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M2, 5);

    addParamToTable(P_KSU_OTKL_VZAD ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M128, 6);
    addParamToTable(P_KSU_OTKL_VZAD ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M128, 6);
    addParamToTable(P_KSU_OTKL_VZAD ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M128, 6);
    addParamToTable(P_KSU_OTKL_VZAD ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M128, 6);

    addParamToTable(P_KSU_D_PSI ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M128, 7);
    addParamToTable(P_KSU_D_PSI ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M128, 7);
    addParamToTable(P_KSU_D_PSI ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M128, 7);
    addParamToTable(P_KSU_D_PSI ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M128, 7);

    addParamToTable(P_KSU_D_H ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M256, 8);
    addParamToTable(P_KSU_D_H ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M256, 8);
    addParamToTable(P_KSU_D_H ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M256, 8);
    addParamToTable(P_KSU_D_H ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M256, 8);

    addParamToTable(P_KSU_RUD_SR ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M32, 9);
    addParamToTable(P_KSU_RUD_SR ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M32, 9);
    addParamToTable(P_KSU_RUD_SR ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M32, 9);
    addParamToTable(P_KSU_RUD_SR ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M32, 9);

    addParamToTable(P_KSU_FI_T,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M40, 10);
    addParamToTable(P_KSU_FI_T,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M40, 10);
    addParamToTable(P_KSU_FI_T,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M40, 10);
    addParamToTable(P_KSU_FI_T,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M40, 10);

    addParamToTable(P_KSU_FI_K,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M20, 11);
    addParamToTable(P_KSU_FI_K,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M20, 11);
    addParamToTable(P_KSU_FI_K,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M20, 11);
    addParamToTable(P_KSU_FI_K,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M20, 11);

    addParamToTable(P_KSU_D_FI_RUD,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M32, 12);
    addParamToTable(P_KSU_D_FI_RUD,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M32, 12);
    addParamToTable(P_KSU_D_FI_RUD,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M32, 12);
    addParamToTable(P_KSU_D_FI_RUD,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M32, 12);

    addParamToTable(P_KSU_ALFA_RUD_P,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M32, 13);
    addParamToTable(P_KSU_ALFA_RUD_P,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M32, 13);
    addParamToTable(P_KSU_ALFA_RUD_P,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M32, 13);
    addParamToTable(P_KSU_ALFA_RUD_P,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M32, 13);

    addParamToTable(P_KSU_ALFA_RUD_L,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M32, 14);
    addParamToTable(P_KSU_ALFA_RUD_L,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M32, 14);
    addParamToTable(P_KSU_ALFA_RUD_L,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M32, 14);
    addParamToTable(P_KSU_ALFA_RUD_L,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M32, 14);

    addParamToTable(P_KSU_GAMMA_ZAD_SAU,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M90, 15);
    addParamToTable(P_KSU_GAMMA_ZAD_SAU,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M90, 15);
    addParamToTable(P_KSU_GAMMA_ZAD_SAU,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M90, 15);
    addParamToTable(P_KSU_GAMMA_ZAD_SAU,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M90, 15);

    addParamToTable(P_KSU_NY_ZAD_SAU,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M2, 16);
    addParamToTable(P_KSU_NY_ZAD_SAU,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M2, 16);
    addParamToTable(P_KSU_NY_ZAD_SAU,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M2, 16);
    addParamToTable(P_KSU_NY_ZAD_SAU,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M2, 16);

    addParamToTable(P_KSU_HSOS,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M16384, 17);
    addParamToTable(P_KSU_HSOS,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M16384, 17);
    addParamToTable(P_KSU_HSOS,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M16384, 17);
    addParamToTable(P_KSU_HSOS,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M16384, 17);

    addParamToTable(P_KSU_TETA_TRAEK,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M90, 18);
    addParamToTable(P_KSU_TETA_TRAEK,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M90, 18);
    addParamToTable(P_KSU_TETA_TRAEK,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M90, 18);
    addParamToTable(P_KSU_TETA_TRAEK,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M90, 18);






    addParamToTable(P_KSU_D11         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M190, 0173);
    addParamToTable(P_KSU_D11         , AR_IN_IKRL_1_KSU, E_P_AR_H28_L14_S0_M190, 0173);
    addParamToTable(P_KSU_D11    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M190, 19);
    addParamToTable(P_KSU_D11    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M190, 19);
    addParamToTable(P_KSU_D11    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M190, 19);
    addParamToTable(P_KSU_D11    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M190, 19);


    addParamToTable(P_KSU_D21         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M190, 0174);
    addParamToTable(P_KSU_D21         , AR_IN_IKRL_1_KSU, E_P_AR_H28_L14_S0_M190, 0174);
    addParamToTable(P_KSU_D21    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M190,20);
    addParamToTable(P_KSU_D21    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M190,20);
    addParamToTable(P_KSU_D21    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M190,20);
    addParamToTable(P_KSU_D21    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M190,20);



    addParamToTable(P_KSU_D12         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M190, 0175);
    addParamToTable(P_KSU_D12         , AR_IN_IKRL_1_KSU, E_P_AR_H28_L14_S0_M190, 0175);
    addParamToTable(P_KSU_D12    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M190,21);
    addParamToTable(P_KSU_D12    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M190,21);
    addParamToTable(P_KSU_D12    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M190,21);
    addParamToTable(P_KSU_D12    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M190,21);


    addParamToTable(P_KSU_D22         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M190, 0176);
    addParamToTable(P_KSU_D22         , AR_IN_IKRL_1_KSU, E_P_AR_H28_L14_S0_M190, 0176);
    addParamToTable(P_KSU_D22    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M190,22);
    addParamToTable(P_KSU_D22    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M190,22);
    addParamToTable(P_KSU_D22    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M190,22);
    addParamToTable(P_KSU_D22    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M190,22);


    addParamToTable(P_KSU_EPS_G         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M0_5, 020);
    addParamToTable(P_KSU_EPS_G         , AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M0_5, 020);

    addParamToTable(P_KSU_EPS_K         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H29_L14_S1_M2,   021);
    addParamToTable(P_KSU_EPS_K         , AR_IN_IKRL_1_KSU, E_P_AR_H29_L14_S1_M2,   021);

    addParamToTable(P_KSU_H_OTN         , AR_OUT_KSU_L_UCOKS1, E_P_AR_H28_L14_S0_M0,   0250);

    addParamToTable(P_KSU_D11           , MIL_OUT_KSU_NVG_SA3, E_P_MIL_H5_L19_S0_M190, 6);
    addParamToTable(P_KSU_D11           , MIL_OUT_KSU_OSO_SA3, E_P_MIL_H5_L19_S0_M190, 6);
    addParamToTable(P_KSU_D12           , MIL_OUT_KSU_NVG_SA3, E_P_MIL_H5_L19_S0_M190, 7);
    addParamToTable(P_KSU_D12           , MIL_OUT_KSU_OSO_SA3, E_P_MIL_H5_L19_S0_M190, 7);
    addParamToTable(P_KSU_D21           , MIL_OUT_KSU_NVG_SA3, E_P_MIL_H5_L19_S0_M190, 8);
    addParamToTable(P_KSU_D21           , MIL_OUT_KSU_OSO_SA3, E_P_MIL_H5_L19_S0_M190, 8);
    addParamToTable(P_KSU_D22           , MIL_OUT_KSU_NVG_SA3, E_P_MIL_H5_L19_S0_M190, 9);
    addParamToTable(P_KSU_D22           , MIL_OUT_KSU_OSO_SA3, E_P_MIL_H5_L19_S0_M190, 9);
    addParamToTable(P_KSU_D_POSH        , MIL_OUT_KSU_NVG_SA3, E_P_MIL_H5_L19_S0_M190, 10);
    addParamToTable(P_KSU_D_POSH        , MIL_OUT_KSU_OSO_SA3, E_P_MIL_H5_L19_S0_M190, 10);
    addParamToTable(P_KSU_D_POSH    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M32,23);
    addParamToTable(P_KSU_D_POSH    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M32,23);
    addParamToTable(P_KSU_D_POSH    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M32,23);
    addParamToTable(P_KSU_D_POSH    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M32,23);

    addParamToTable(P_KSU_VRULEG    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M160,24);
    addParamToTable(P_KSU_VRULEG    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M160,24);
    addParamToTable(P_KSU_VRULEG    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M160,24);
    addParamToTable(P_KSU_VRULEG    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M160,24);

    addParamToTable(P_KSU_XTPLINT    ,     MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M16,25);
    addParamToTable(P_KSU_XTPLINT    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M16,25);
    addParamToTable(P_KSU_XTPLINT    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M16,25);
    addParamToTable(P_KSU_XTPLINT    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M16,25);

    addParamToTable(P_KSU_XTPRINT    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S1_M16,26);
    addParamToTable(P_KSU_XTPRINT    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S1_M16,26);
    addParamToTable(P_KSU_XTPRINT    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S1_M16,26);
    addParamToTable(P_KSU_XTPRINT    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S1_M16,26);


    addParamToTable(P_KSU_KOD_FROM_KSU    ,      MIL_OUT_KSU58_NVG1_SA3,E_P_MIL_H4_L19_S0_M0,31);
    addParamToTable(P_KSU_KOD_FROM_KSU    ,      MIL_OUT_KSU58_NVG2_SA3,E_P_MIL_H4_L19_S0_M0,31);
    addParamToTable(P_KSU_KOD_FROM_KSU    ,      MIL_OUT_KSU58_BP1_SA3, E_P_MIL_H4_L19_S0_M0,31);
    addParamToTable(P_KSU_KOD_FROM_KSU    ,      MIL_OUT_KSU58_BP2_SA3, E_P_MIL_H4_L19_S0_M0,31);


    addParamToTable(P_KSU_XTET           , MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M6, 7);
    addParamToTable(P_KSU_XTET           , MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M6, 7);
    addParamToTable(P_KSU_XTET		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M64, 14);
    addParamToTable(P_KSU_XTET		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M64, 14);
    addParamToTable(P_KSU_XTET		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M64, 14);
    addParamToTable(P_KSU_XTET		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M64, 14);
    addParamToTable(P_KSU_XTET      , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M64, 14);


    addParamToTable(P_KSU_XGAMM           , MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M8, 8);
    addParamToTable(P_KSU_XGAMM           , MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M8, 8);
    addParamToTable(P_KSU_XGAMM		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 15);
    addParamToTable(P_KSU_XGAMM		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 15);
    addParamToTable(P_KSU_XGAMM		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 15);
    addParamToTable(P_KSU_XGAMM		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 15);



    addParamToTable(P_KSU_XP           , MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M32, 9);
    addParamToTable(P_KSU_XP           , MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M32, 9);
    addParamToTable(P_KSU_XP		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 7);
    addParamToTable(P_KSU_XP		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 7);
    addParamToTable(P_KSU_XP		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 7);
    addParamToTable(P_KSU_XP		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 7);


    addParamToTable(P_KSU_CPVO_LEV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 8);
    addParamToTable(P_KSU_CPVO_LEV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 8);
    addParamToTable(P_KSU_CPVO_LEV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 8);
    addParamToTable(P_KSU_CPVO_LEV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 8);

    addParamToTable(P_KSU_CPVO_PRAV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 9);
    addParamToTable(P_KSU_CPVO_PRAV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 9);
    addParamToTable(P_KSU_CPVO_PRAV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 9);
    addParamToTable(P_KSU_CPVO_PRAV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 9);

    addParamToTable(P_KSU_FLL	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 10);
    addParamToTable(P_KSU_FLL	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 10);
    addParamToTable(P_KSU_FLL	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 10);
    addParamToTable(P_KSU_FLL	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 10);
    addParamToTable(P_KSU_FLP	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 11);
    addParamToTable(P_KSU_FLP	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 11);
    addParamToTable(P_KSU_FLP	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 11);
    addParamToTable(P_KSU_FLP	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 11);

    addParamToTable(P_KSU_STABL	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 12);
    addParamToTable(P_KSU_STABL	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 12);
    addParamToTable(P_KSU_STABL	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 12);
    addParamToTable(P_KSU_STABL	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 12);

    addParamToTable(P_KSU_STABP	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 13);
    addParamToTable(P_KSU_STABP	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 13);
    addParamToTable(P_KSU_STABP	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 13);
    addParamToTable(P_KSU_STABP	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 13);

    addParamToTable(P_KSU_NAPL_LEV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 16);
    addParamToTable(P_KSU_NAPL_LEV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 16);
    addParamToTable(P_KSU_NAPL_LEV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 16);
    addParamToTable(P_KSU_NAPL_LEV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 16);

    addParamToTable(P_KSU_NAPL_PRAV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 17);
    addParamToTable(P_KSU_NAPL_PRAV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 17);
    addParamToTable(P_KSU_NAPL_PRAV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 17);
    addParamToTable(P_KSU_NAPL_PRAV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 17);

    addParamToTable(P_KSU_NOS_LEV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 18);
    addParamToTable(P_KSU_NOS_LEV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 18);
    addParamToTable(P_KSU_NOS_LEV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 18);
    addParamToTable(P_KSU_NOS_LEV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 18);

    addParamToTable(P_KSU_NOS_PRAV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 19);
    addParamToTable(P_KSU_NOS_PRAV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 19);
    addParamToTable(P_KSU_NOS_PRAV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 19);
    addParamToTable(P_KSU_NOS_PRAV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 19);

    addParamToTable(P_KSU_EL_LEV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 20);
    addParamToTable(P_KSU_EL_LEV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 20);
    addParamToTable(P_KSU_EL_LEV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 20);
    addParamToTable(P_KSU_EL_LEV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 20);

    addParamToTable(P_KSU_EL_PRAV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M16, 21);
    addParamToTable(P_KSU_EL_PRAV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M16, 21);
    addParamToTable(P_KSU_EL_PRAV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M16, 21);
    addParamToTable(P_KSU_EL_PRAV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M16, 21);

    addParamToTable(P_KSU_PS_LEV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M8, 22);
    addParamToTable(P_KSU_PS_LEV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M8, 22);
    addParamToTable(P_KSU_PS_LEV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M8, 22);
    addParamToTable(P_KSU_PS_LEV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M8, 22);

    addParamToTable(P_KSU_PS_PRAV	     , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M8, 23);
    addParamToTable(P_KSU_PS_PRAV	     , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M8, 23);
    addParamToTable(P_KSU_PS_PRAV	     , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M8, 23);
    addParamToTable(P_KSU_PS_PRAV	     , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M8, 23);

    addParamToTable(P_KSU_H_BEZ	             , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M8192, 24);
    addParamToTable(P_KSU_H_BEZ	             , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M8192, 24);
    addParamToTable(P_KSU_H_BEZ	             , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M8192, 24);
    addParamToTable(P_KSU_H_BEZ	             , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M8192, 24);

    addParamToTable(P_KSU_NY_RASP_DOP	             , MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M5, 25);
    addParamToTable(P_KSU_NY_RASP_DOP	             , MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M5, 25);
    addParamToTable(P_KSU_NY_RASP_DOP	             , MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M5, 25);
    addParamToTable(P_KSU_NY_RASP_DOP	             , MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M5, 25);







    addParamToTable(P_KSU_ALF_RUD           , MIL_OUT_KSU_NVG_SA2, E_P_MIL_H5_L19_S0_M32, 10);
    addParamToTable(P_KSU_ALF_RUD           , MIL_OUT_KSU_OSO_SA2, E_P_MIL_H5_L19_S0_M32, 10);

    addParamToTable(P_KSU_XTPL           , MIL_OUT_KSU_NVG_SA2, E_P_MIL_H5_L19_S0_M16, 11);
    addParamToTable(P_KSU_XTPL           , MIL_OUT_KSU_OSO_SA2, E_P_MIL_H5_L19_S0_M16, 11);

    addParamToTable(P_KSU_XTPR           , MIL_OUT_KSU_NVG_SA2, E_P_MIL_H5_L19_S0_M16, 12);
    addParamToTable(P_KSU_XTPR           , MIL_OUT_KSU_OSO_SA2, E_P_MIL_H5_L19_S0_M16, 12);

    addParamToTable(P_KSU_OMEGA_X		, MIL_OUT_KSU_NVG_SA2,   E_P_MIL_H4_L19_S1_M90, 4);
    addParamToTable(P_KSU_OMEGA_X		, MIL_OUT_KSU_OSO_SA2,   E_P_MIL_H4_L19_S1_M90, 4);

    addParamToTable(P_KSU_OMEGA_X		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M90, 1);
    addParamToTable(P_KSU_OMEGA_X		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M90, 1);
    addParamToTable(P_KSU_OMEGA_X		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M90, 1);
    addParamToTable(P_KSU_OMEGA_X		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M90, 1);

    addParamToTable(P_KSU_OMEGA_Y		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M32, 5);
    addParamToTable(P_KSU_OMEGA_Y		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M32, 5);

    addParamToTable(P_KSU_OMEGA_Y		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 2);
    addParamToTable(P_KSU_OMEGA_Y		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 2);
    addParamToTable(P_KSU_OMEGA_Y		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 2);
    addParamToTable(P_KSU_OMEGA_Y		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 2);

    addParamToTable(P_KSU_OMEGA_Z		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S1_M32, 6);
    addParamToTable(P_KSU_OMEGA_Z		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S1_M32, 6);
    addParamToTable(P_KSU_OMEGA_Z		, MIL_OUT_KSU58_NVG1_SA2,E_P_MIL_H4_L19_S1_M32, 3);
    addParamToTable(P_KSU_OMEGA_Z		, MIL_OUT_KSU58_NVG2_SA2,E_P_MIL_H4_L19_S1_M32, 3);
    addParamToTable(P_KSU_OMEGA_Z		, MIL_OUT_KSU58_BP1_SA2, E_P_MIL_H4_L19_S1_M32, 3);
    addParamToTable(P_KSU_OMEGA_Z		, MIL_OUT_KSU58_BP2_SA2, E_P_MIL_H4_L19_S1_M32, 3);

    addParamToTable(P_KSU_SRK4		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK4		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK4          , AR_IN_KSU_UCOKS1,    E_P_AR_H28_L1_S0_M0, 04);
    addParamToTable(P_KSU_SRK4          , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0, 04);

    addParamToTable(P_KSU_SRK4		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK4		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK4		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK4		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 26);

    addParamToTable(P_KSU_SRK4_MAT          , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 04);


    addParamToTable(P_KSU_SRK5		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S0_M0, 22);
    addParamToTable(P_KSU_SRK5		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S0_M0, 22);
    addParamToTable(P_KSU_SRK5		, AR_IN_KSU_UCOKS1, E_P_AR_H28_L1_S0_M0, 010);
    addParamToTable(P_KSU_SRK5      , AR_OUT_KSU_L_UCOKS1, E_P_AR_H32_L1_S0_M0, 05);

    addParamToTable(P_KSU_SRK5	, MIL_OUT_KSU58_NVG1_SA3,   E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK5	, MIL_OUT_KSU58_NVG2_SA3,   E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK5	, MIL_OUT_KSU58_BP1_SA3,    E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK5	, MIL_OUT_KSU58_BP2_SA3,    E_P_MIL_H4_L19_S0_M0, 28);

    addParamToTable(P_KSU_SRK5_MAT          , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 05);

    addParamToTable(P_KSU_SRK18	, MIL_OUT_KSU58_NVG1_SA3,   E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK18	, MIL_OUT_KSU58_NVG2_SA3,   E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK18	, MIL_OUT_KSU58_BP1_SA3,    E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK18	, MIL_OUT_KSU58_BP2_SA3,    E_P_MIL_H4_L19_S0_M0, 29);

    addParamToTable(P_KSU_SRK19	, MIL_OUT_KSU58_NVG1_SA3,   E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK19	, MIL_OUT_KSU58_NVG2_SA3,   E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK19	, MIL_OUT_KSU58_BP1_SA3,    E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK19	, MIL_OUT_KSU58_BP2_SA3,    E_P_MIL_H4_L19_S0_M0, 30);

    addParamToTable(P_KSU_SRK7		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK7		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S0_M0, 23);
    addParamToTable(P_KSU_SRK7      , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0,06 );

    addParamToTable(P_KSU_SRK7		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK7		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK7		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK7		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK7_MAT          , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 06);

    addParamToTable(P_KSU_SRK8		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK8		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK8      , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0,011);

    addParamToTable(P_KSU_SRK8		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK8		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK8		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK8		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK8_MAT          , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 011);

    addParamToTable(P_KSU_SRK53		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK53		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S0_M0, 25);
    addParamToTable(P_KSU_SRK54		, MIL_OUT_KSU_NVG_SA2, E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK54		, MIL_OUT_KSU_OSO_SA2, E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_CNT_ISPR_KSU  , MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_KSU_CNT_ISPR_KSU  , MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 12);


    addParamToTable(P_KSU_CNT_ISPR_KSU		, MIL_OUT_KSU58_NVG1_SA3,  E_P_MIL_H4_L19_S0_M0 , 27);
    addParamToTable(P_KSU_CNT_ISPR_KSU		, MIL_OUT_KSU58_NVG2_SA3,  E_P_MIL_H4_L19_S0_M0 , 27);
    addParamToTable(P_KSU_CNT_ISPR_KSU		, MIL_OUT_KSU58_BP1_SA3,   E_P_MIL_H4_L19_S0_M0 , 27);
    addParamToTable(P_KSU_CNT_ISPR_KSU		, MIL_OUT_KSU58_BP2_SA3,   E_P_MIL_H4_L19_S0_M0 , 27);


    addParamToTable(P_KSU_SRK10		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_KSU_SRK10		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_KSU_SRK10     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H32_L1_S0_M0,  012);

    addParamToTable(P_KSU_SRK10		, MIL_OUT_KSU58_NVG1_SA2,   E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK10		, MIL_OUT_KSU58_NVG2_SA2,   E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK10		, MIL_OUT_KSU58_BP1_SA2,    E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK10		, MIL_OUT_KSU58_BP2_SA2,    E_P_MIL_H4_L19_S0_M0, 26);
    addParamToTable(P_KSU_SRK10_MAT          , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 012);

    addParamToTable(P_KSU_SRK16		, MIL_OUT_KSU58_NVG1_SA2,   E_P_MIL_H4_L19_S0_M0, 27);
    addParamToTable(P_KSU_SRK16		, MIL_OUT_KSU58_NVG2_SA2,   E_P_MIL_H4_L19_S0_M0, 27);
    addParamToTable(P_KSU_SRK16		, MIL_OUT_KSU58_BP1_SA2,    E_P_MIL_H4_L19_S0_M0, 27);
    addParamToTable(P_KSU_SRK16		, MIL_OUT_KSU58_BP2_SA2,    E_P_MIL_H4_L19_S0_M0, 27);

    addParamToTable(P_KSU_SRK17		, MIL_OUT_KSU58_NVG1_SA2,   E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK17		, MIL_OUT_KSU58_NVG2_SA2,   E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK17		, MIL_OUT_KSU58_BP1_SA2,    E_P_MIL_H4_L19_S0_M0, 28);
    addParamToTable(P_KSU_SRK17		, MIL_OUT_KSU58_BP2_SA2,    E_P_MIL_H4_L19_S0_M0, 28);

    addParamToTable(P_KSU_SRK9		, MIL_OUT_KSU58_NVG1_SA2,   E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK9		, MIL_OUT_KSU58_NVG2_SA2,   E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK9		, MIL_OUT_KSU58_BP1_SA2,    E_P_MIL_H4_L19_S0_M0, 29);
    addParamToTable(P_KSU_SRK9		, MIL_OUT_KSU58_BP2_SA2,    E_P_MIL_H4_L19_S0_M0, 29);

    addParamToTable(P_KSU_SRK21		, MIL_OUT_KSU58_NVG1_SA2,   E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK21		, MIL_OUT_KSU58_NVG2_SA2,   E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK21		, MIL_OUT_KSU58_BP1_SA2,    E_P_MIL_H4_L19_S0_M0, 30);
    addParamToTable(P_KSU_SRK21		, MIL_OUT_KSU58_BP2_SA2,    E_P_MIL_H4_L19_S0_M0, 30);

    addParamToTable(P_KSU_SRK22		, MIL_OUT_KSU58_NVG1_SA2,   E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_KSU_SRK22		, MIL_OUT_KSU58_NVG2_SA2,   E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_KSU_SRK22		, MIL_OUT_KSU58_BP1_SA2,    E_P_MIL_H4_L19_S0_M0, 31);
    addParamToTable(P_KSU_SRK22		, MIL_OUT_KSU58_BP2_SA2,    E_P_MIL_H4_L19_S0_M0, 31);

    addParamToTable(P_KSU_SRK11		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_KSU_SRK11		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_KSU_SRK11     , AR_OUT_KSU_L_UCOKS1, E_P_AR_H32_L1_S0_M0,  013);
    addParamToTable(P_KSU_SRK11_MAT          , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 013);

    addParamToTable(P_KSU_SRK12		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_KSU_SRK12		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 15);

    addParamToTable(P_KSU_SRK12     , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0, 014);
    addParamToTable(P_KSU_SRK12_MAT     , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 014);

    addParamToTable(P_KSU_SRK13     , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H32_L1_S0_M0, 015);
    addParamToTable(P_KSU_SRK13_MAT     , AR_OUT_KSU_L_UCOKS1,    E_P_AR_H31_L30_S0_M0, 015);



    addParamToTable(P_KSU_SRK55		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 16);
    addParamToTable(P_KSU_SRK55		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 16);

    addParamToTable(P_KSU_SRK56		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 17);
    addParamToTable(P_KSU_SRK56		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 17);


    addParamToTable(P_KSU_SRK57		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 18);
    addParamToTable(P_KSU_SRK57		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 18);

    addParamToTable(P_KSU_SRK58		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 19);
    addParamToTable(P_KSU_SRK58		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 19);

    addParamToTable(P_KSU_SRK59		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_KSU_SRK59		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 20);

    addParamToTable(P_KSU_SRK60		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK60		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 21);

    addParamToTable(P_KSU_SRK61		, MIL_OUT_KSU_NVG_SA3, E_P_MIL_H4_L19_S0_M0, 22);
    addParamToTable(P_KSU_SRK61		, MIL_OUT_KSU_OSO_SA3, E_P_MIL_H4_L19_S0_M0, 22);

     addParamToTable(P_KSU_SRK6		, AR_IN_KSU_UCOKS1, E_P_AR_H28_L1_S0_M0, 011);
     addParamToTable(P_KSU_SRK6		, AR_OUT_IKRL_1_KSU_1, E_P_AR_H28_L1_S0_M0, 011);

     addParamToTable(P_KSU_SRK6		, MIL_OUT_KSU58_NVG1_SA1,   E_P_MIL_H4_L19_S0_M0, 27);
     addParamToTable(P_KSU_SRK6		, MIL_OUT_KSU58_NVG2_SA1,   E_P_MIL_H4_L19_S0_M0, 27);
     addParamToTable(P_KSU_SRK6		, MIL_OUT_KSU58_BP1_SA1,    E_P_MIL_H4_L19_S0_M0, 27);
     addParamToTable(P_KSU_SRK6		, MIL_OUT_KSU58_BP2_SA1,    E_P_MIL_H4_L19_S0_M0, 27);

    addParamToTable(P_KSU_XP        ,AR_IN_KSU_UCOKS1,E_P_AR_H29_L14_S1_M32,05);
    addParamToTable(P_KSU_XP_MAT ,AR_IN_KSU_UCOKS1,E_P_AR_H31_L30_S0_M0, 05);
    addParamToTable(P_KSU_XTET      ,AR_IN_KSU_UCOKS1,E_P_AR_H29_L14_S1_M6, 0171);
    addParamToTable(P_KSU_XTET_MAT ,AR_IN_KSU_UCOKS1,E_P_AR_H31_L30_S0_M0, 0171);
    addParamToTable(P_KSU_XGAMM     ,AR_IN_KSU_UCOKS1,E_P_AR_H29_L14_S1_M8, 0172);
    addParamToTable(P_KSU_XGAMM_MAT ,AR_IN_KSU_UCOKS1,E_P_AR_H31_L30_S0_M0, 0172);

    addParamToTable(P_KSU_ALF_RUD   ,AR_IN_KSU_UCOKS1,E_P_AR_H29_L14_S1_M64,055);
    addParamToTable(P_KSU_ALF_RUD_MAT ,AR_IN_KSU_UCOKS1,E_P_AR_H31_L30_S0_M0, 055);


    addParamToTable(P_KSU_SRK30	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 4);
    addParamToTable(P_KSU_SRK30	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 4);
    addParamToTable(P_KSU_SRK30	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 4);
    addParamToTable(P_KSU_SRK30	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 4);

    addParamToTable(P_KSU_SRK31	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 5);
    addParamToTable(P_KSU_SRK31	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 5);
    addParamToTable(P_KSU_SRK31	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 5);
    addParamToTable(P_KSU_SRK31	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 5);

    addParamToTable(P_KSU_SRK32	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 6);
    addParamToTable(P_KSU_SRK32	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 6);
    addParamToTable(P_KSU_SRK32	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 6);
    addParamToTable(P_KSU_SRK32	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 6);

    addParamToTable(P_KSU_SRK33	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 7);
    addParamToTable(P_KSU_SRK33	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 7);
    addParamToTable(P_KSU_SRK33	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 7);
    addParamToTable(P_KSU_SRK33	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 7);

    addParamToTable(P_KSU_SRK34	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 8);
    addParamToTable(P_KSU_SRK34	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 8);
    addParamToTable(P_KSU_SRK34	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 8);
    addParamToTable(P_KSU_SRK34	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 8);

    addParamToTable(P_KSU_SRK35	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 9);
    addParamToTable(P_KSU_SRK35	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 9);
    addParamToTable(P_KSU_SRK35	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 9);
    addParamToTable(P_KSU_SRK35	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 9);

    addParamToTable(P_KSU_SRK36	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 10);
    addParamToTable(P_KSU_SRK36	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 10);
    addParamToTable(P_KSU_SRK36	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 10);
    addParamToTable(P_KSU_SRK36	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 10);

    addParamToTable(P_KSU_SRK37	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 11);
    addParamToTable(P_KSU_SRK37	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 11);
    addParamToTable(P_KSU_SRK37	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 11);
    addParamToTable(P_KSU_SRK37	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 11);

    addParamToTable(P_KSU_SRK38	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_KSU_SRK38	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_KSU_SRK38	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_KSU_SRK38	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 12);

    addParamToTable(P_KSU_SRK39	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_KSU_SRK39	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_KSU_SRK39	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_KSU_SRK39	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 13);

    addParamToTable(P_KSU_SRK40	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_KSU_SRK40	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_KSU_SRK40	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_KSU_SRK40	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 14);

    addParamToTable(P_KSU_SRK41	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_KSU_SRK41	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_KSU_SRK41	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_KSU_SRK41	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 15);

    addParamToTable(P_KSU_SRK42	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 16);
    addParamToTable(P_KSU_SRK42	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 16);
    addParamToTable(P_KSU_SRK42	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 16);
    addParamToTable(P_KSU_SRK42	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 16);

    addParamToTable(P_KSU_SRK43	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 17);
    addParamToTable(P_KSU_SRK43	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 17);
    addParamToTable(P_KSU_SRK43	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 17);
    addParamToTable(P_KSU_SRK43	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 17);

    addParamToTable(P_KSU_SRK44	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 18);
    addParamToTable(P_KSU_SRK44	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 18);
    addParamToTable(P_KSU_SRK44	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 18);
    addParamToTable(P_KSU_SRK44	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 18);

    addParamToTable(P_KSU_SRK45	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 19);
    addParamToTable(P_KSU_SRK45	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 19);
    addParamToTable(P_KSU_SRK45	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 19);
    addParamToTable(P_KSU_SRK45	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 19);

    addParamToTable(P_KSU_SRK46	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_KSU_SRK46	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_KSU_SRK46	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 20);
    addParamToTable(P_KSU_SRK46	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 20);

    addParamToTable(P_KSU_SRK47	, MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK47	, MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK47	, MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 21);
    addParamToTable(P_KSU_SRK47	, MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 21);

    addParamToTable(P_KSU_SRK01_70 , MIL_OUT_KSU58_NVG1_SA4,   E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK01_70 , MIL_OUT_KSU58_NVG2_SA4,   E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK01_70 , MIL_OUT_KSU58_BP1_SA4,    E_P_MIL_H4_L19_S0_M0, 24);
    addParamToTable(P_KSU_SRK01_70 , MIL_OUT_KSU58_BP2_SA4,    E_P_MIL_H4_L19_S0_M0, 24);

    addParamToTable(P_KSU_RV_H      , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 017);
    addParamToTable(P_KSU_KREN      , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 022);
    addParamToTable(P_KSU_TAN       , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 023);
    addParamToTable(P_KSU_FI_SENI   , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 025);
    addParamToTable(P_KSU_LAM_SENI  , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 027);
    addParamToTable(P_KSU_P_Z       , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 032);

    addParamToTable(P_KSU_RV_H_MAT      , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 017);
    addParamToTable(P_KSU_KREN_MAT      , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 022);
    addParamToTable(P_KSU_TAN_MAT       , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 023);
    addParamToTable(P_KSU_FI_SENI_MAT   , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 025);
    addParamToTable(P_KSU_LAM_SENI_MAT  , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 027);
    addParamToTable(P_KSU_P_Z_MAT       , AR_OUT_IKRL_1_KSU_1,    E_P_AR_H28_L1_S0_M0, 032);




   addParamToTable( P_VSU_KSL1     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0111);
   addParamToTable( P_VSU_KSL2     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0266);
   addParamToTable( P_VSU_KSL1     , AR_OUT_VSU_UCOKS, E_P_AR_H31_L14_S0_M0, 0111);
   addParamToTable( P_VSU_KSL2     , AR_OUT_VSU_UCOKS, E_P_AR_H31_L14_S0_M0, 0266);
   addParamToTable( P_VSU_SBIX1    , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0167);
   addParamToTable( P_VSU_SBIX2    , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0171);
   addParamToTable( P_VSU_SBIX1    , AR_OUT_VSU_UCOKS, E_P_AR_H31_L14_S0_M0, 0167);
      addParamToTable( P_VSU_SBIX2    , AR_OUT_VSU_UCOKS, E_P_AR_H31_L14_S0_M0, 0171);
      
   addParamToTable( P_VSU_DCR      , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0272);
   addParamToTable( P_VSU_DCR      , AR_OUT_VSU_UCOKS, E_P_AR_H31_L14_S0_M0, 0272);
   addParamToTable( P_VSU_DCSKD    , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0273);
   addParamToTable( P_VSU_DCSKD    , AR_OUT_VSU_UCOKS, E_P_AR_H31_L14_S0_M0, 0273);
   addParamToTable( P_VSU_DCKU     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0371);
   addParamToTable( P_VSU_SBX1     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0163);
   addParamToTable( P_VSU_SBX2     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0164);
   addParamToTable( P_VSU_SBX3     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0165);
   addParamToTable( P_VSU_SBX3     , AR_OUT_VSU_UCOKS, E_P_AR_H31_L14_S0_M0, 0165);
   addParamToTable( P_VSU_SOO      , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0352);
   addParamToTable( P_VSU_SOU      , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0353);
   addParamToTable( P_VSU_SOFU1    , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0160);
   addParamToTable( P_VSU_SOFU2    , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0161);
   addParamToTable( P_VSU_SOFU3    , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0170);
   addParamToTable( P_VSU_SSUPIT   , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0351);
   addParamToTable( P_VSU_SOLD     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0350);
   addParamToTable( P_VSU_CPO      , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0104);
   addParamToTable( P_VSU_NAGR     , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0145);
   addParamToTable( P_VSU_NDVIG    , AR_OUT_VSU_BKS3, E_P_AR_H31_L14_S0_M0, 0115);



    addParamToTable(P_VSU_NTK      , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M81_92  , 0176    );
    addParamToTable(P_VSU_NTK      , AR_OUT_VSU_UCOKS, E_P_AR_H29_L14_S1_M81_92  , 0176    );    
    addParamToTable(P_VSU_MC_NTK   , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 0176    );
    addParamToTable(P_VSU_MC_NTK   , AR_OUT_VSU_UCOKS, E_P_AR_H31_L30_S0_M0      , 0176    );
    addParamToTable(P_VSU_NTK1     , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M81_92  , 05      );    
    addParamToTable(P_VSU_MC_NTK1  , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 05      );
    addParamToTable(P_VSU_NTK2     , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M81_92  , 06      );
    addParamToTable(P_VSU_MC_NTK2  , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 06      );
    addParamToTable(P_VSU_TG       , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M4096   , 0175    );
    addParamToTable(P_VSU_TG       , AR_OUT_VSU_UCOKS, E_P_AR_H29_L14_S1_M81_92   , 0175    );
    
    addParamToTable(P_VSU_MC_TG    , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 0175    );
    addParamToTable(P_VSU_MC_TG    , AR_OUT_VSU_UCOKS, E_P_AR_H31_L30_S0_M0      , 0175    );
    addParamToTable(P_VSU_TG1      , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M4096   , 024     );
    addParamToTable(P_VSU_MC_TG1   , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 024     );
    addParamToTable(P_VSU_TG2      , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M4096   , 025     );
    addParamToTable(P_VSU_MC_TG2   , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 025     );
    addParamToTable(P_VSU_TGX      , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M2048   , 037     );
    addParamToTable(P_VSU_MC_TGX   , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 037     );

   addParamToTable(P_VSU_TM        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M2048   , 0316    );
   addParamToTable(P_VSU_TM        , AR_OUT_VSU_UCOKS, E_P_AR_H29_L14_S1_M2048   , 0316    );
   addParamToTable(P_VSU_MC_TM     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 0316    );
   addParamToTable(P_VSU_MC_TM     , AR_OUT_VSU_UCOKS, E_P_AR_H31_L30_S0_M0      , 0316    );

   addParamToTable(P_VSU_PMF           , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M64    , 07          );
   addParamToTable(P_VSU_PMF           , AR_OUT_VSU_UCOKS, E_P_AR_H29_L14_S1_M64    , 07          );
   addParamToTable(P_VSU_MC_PMF        , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 07          );
   addParamToTable(P_VSU_MC_PMF        , AR_OUT_VSU_UCOKS, E_P_AR_H31_L30_S0_M0     , 07          );
   addParamToTable(P_VSU_PMN           , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M64    , 0317        );
   addParamToTable(P_VSU_MC_PMN        , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0317        );
            addParamToTable(P_VSU_PT            , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M64    , 0320        );
            addParamToTable(P_VSU_MC_PT         , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0320        );
            addParamToTable(P_VSU_TAGR          , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M1024  , 0226        );
            addParamToTable(P_VSU_MC_TAGR       , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0226        );
            addParamToTable(P_VSU_GT            , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M256   , 0244        );
            addParamToTable(P_VSU_MC_GT         , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0244        );
            addParamToTable(P_VSU_GT            , AR_OUT_VSU_UCOKS, E_P_AR_H29_L14_S1_M256   , 0244        );
                       addParamToTable(P_VSU_MC_GT         , AR_OUT_VSU_UCOKS, E_P_AR_H31_L30_S0_M0     , 0244        );
                      
            addParamToTable(P_VSU_TGS           , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M4096  , 016         );
            addParamToTable(P_VSU_MC_TGS        , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 016         );
            addParamToTable(P_VSU_NARERRD       , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 020         );
            addParamToTable(P_VSU_MC_NARERRD    , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 020         );
            addParamToTable(P_VSU_NARAPU        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 0113        );
            addParamToTable(P_VSU_MC_NARAPU     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0113        );
            addParamToTable(P_VSU_KOLZAP        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 0124        );
            addParamToTable(P_VSU_MC_KOLZAP     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0124        );
            addParamToTable(P_VSU_TZAP          , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M431_16, 0216        );
            addParamToTable(P_VSU_MC_TZAP       , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0216        );

           addParamToTable(P_VSU_TGOGR          , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M4096  , 0150        );
           addParamToTable(P_VSU_MC_TGOGR       , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0150        );
           addParamToTable(P_VSU_TGZASCH        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M4096  , 0153        );
           addParamToTable(P_VSU_MC_TGZASCH     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0153        );
           addParamToTable(P_VSU_YGT            , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M1     , 0303        );
           addParamToTable(P_VSU_MC_YGT         , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0303        );
           addParamToTable(P_VSU_IGT            , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M8     , 0304        );
           addParamToTable(P_VSU_MC_IGT         , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0304        );
           addParamToTable(P_VSU_UGT            , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M40_02 , 0305        );
           addParamToTable(P_VSU_MC_UGT         , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0305        );
           addParamToTable(P_VSU_UPIT           , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M1233_73, 0107       );
           addParamToTable(P_VSU_MC_UPIT        , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0107        );
           addParamToTable(P_VSU_U10            , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M259_36, 0306        );
           addParamToTable(P_VSU_MC_U10         , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0306        );
           addParamToTable(P_VSU_NTKZ           , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M81_92 , 0300        );
           addParamToTable(P_VSU_MC_NTKZ        , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0300        );
           addParamToTable(P_VSU_DNTKZ          , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M81_92 , 0301        );
           addParamToTable(P_VSU_MC_DNTKZ       , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0301        );
           addParamToTable(P_VSU_NTKS           , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M81_92 , 0374        );
           addParamToTable(P_VSU_MC_NTKS        , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0374        );
           addParamToTable(P_VSU_GTZ            , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M256   , 0302        );
           addParamToTable(P_VSU_MC_GTZ         , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0302        );
           addParamToTable(P_VSU_TVIB           , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_431_16 , 0214        );
           addParamToTable(P_VSU_MC_TVIB        , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 0214        );
           addParamToTable(P_VSU_NEISPR1        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 060         );
           addParamToTable(P_VSU_MC_NEISPR1     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 060         );
           addParamToTable(P_VSU_NARDV1         , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 061         );
           addParamToTable(P_VSU_MC_NARDV1      , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 061         );
           addParamToTable(P_VSU_KOLZAP1        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 062         );
           addParamToTable(P_VSU_MC_KOLZAP1     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 062         );
           addParamToTable(P_VSU_NEISPR2        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 063         );
           addParamToTable(P_VSU_MC_NEISPR2     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 063         );
           addParamToTable(P_VSU_NARDV2         , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0     , 064         );
           addParamToTable(P_VSU_MC_NARDV2      , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0     , 064         );
           addParamToTable(P_VSU_KOLZAP2       , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 065         );
           addParamToTable(P_VSU_MC_KOLZAP2    , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 065         );
           addParamToTable(P_VSU_NARDV3        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 067         );
           addParamToTable(P_VSU_MC_NARDV3     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 067         );
           addParamToTable(P_VSU_KOLZAP3       , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 070         );
           addParamToTable(P_VSU_MC_KOLZAP3    , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 070         );
           addParamToTable(P_VSU_NARDV4        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 072         );
           addParamToTable(P_VSU_MC_NARDV4     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 072         );
           addParamToTable(P_VSU_KOLZAP4       , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 073         );
           addParamToTable(P_VSU_MC_KOLZAP4    , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 073         );
           addParamToTable(P_VSU_NEISPRP5      , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 074         );
           addParamToTable(P_VSU_MC_NEISPRP5   , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 074         );
           addParamToTable(P_VSU_NEISPRP3      , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 066         );
           addParamToTable(P_VSU_MC_NEISPRP3   , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 066         );
           addParamToTable(P_VSU_NEISPRP4      , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 071         );
           addParamToTable(P_VSU_MC_NEISPRP4   , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 071         );
           addParamToTable(P_VSU_NARDV5        , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 075         );
           addParamToTable(P_VSU_MC_NARDV5     , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 075         );
           addParamToTable(P_VSU_KOLZAP5       , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 076         );
           addParamToTable(P_VSU_MC_KOLZAP5    , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 076         );
           addParamToTable(P_VSU_NEISPRP       , AR_OUT_VSU_BKS3, E_P_AR_H29_L14_S1_M0      , 077         );
           addParamToTable(P_VSU_MC_NEISPRP    , AR_OUT_VSU_BKS3, E_P_AR_H31_L30_S0_M0      , 077         );

    addParamToTable(P_BINS_CY1		, MIL_IN_BINS1_SP2_SA1, E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_CY1		, MIL_IN_BINS2_SP2_SA1, E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_CY1		, MIL_IN_BINS3_SP2_SA1, E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_CY2		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_CY2		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_CY2		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 1);
    addParamToTable(P_BINS_CD2		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_CD2		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_CD2		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 2);
    addParamToTable(P_BINS_FI_IN		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 3);
    addParamToTable(P_BINS_FI_IN		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 3);
    addParamToTable(P_BINS_FI_IN		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 3);
    addParamToTable(P_BINS_LAMBDA_IN		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 5);
    addParamToTable(P_BINS_LAMBDA_IN		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 5);
    addParamToTable(P_BINS_LAMBDA_IN		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L32_S1_M90, 5);
    addParamToTable(P_BINS_HOURB		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L8_S0_M0, 7);
    addParamToTable(P_BINS_HOURB		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L8_S0_M0, 7);
    addParamToTable(P_BINS_HOURB		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L8_S0_M0, 7);
    addParamToTable(P_BINS_MINB		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L14_S0_M0, 7);
    addParamToTable(P_BINS_MINB		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L14_S0_M0, 7);
    addParamToTable(P_BINS_MINB		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L14_S0_M0, 7);
    addParamToTable(P_BINS_SECB		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L9_S0_M0, 8);
    addParamToTable(P_BINS_SECB		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L9_S0_M0, 8);
    addParamToTable(P_BINS_SECB		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L9_S0_M0, 8);
    addParamToTable(P_BINS_DATB		, MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 9);
    addParamToTable(P_BINS_DATB		, MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 9);
    addParamToTable(P_BINS_DATB		, MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 9);
    addParamToTable(P_BINS_ALGWORDS_0           , MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 10);
    addParamToTable(P_BINS_ALGWORDS_0           , MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 10);
    addParamToTable(P_BINS_ALGWORDS_0           , MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 10);
    addParamToTable(P_BINS_ALGWORDS_1           , MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 11);
    addParamToTable(P_BINS_ALGWORDS_1           , MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 11);
    addParamToTable(P_BINS_ALGWORDS_1           , MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 11);
    addParamToTable(P_BINS_ALGWORDS_2           , MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_BINS_ALGWORDS_2           , MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_BINS_ALGWORDS_2           , MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_BINS_ALGWORDS_3           , MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_BINS_ALGWORDS_3           , MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_BINS_ALGWORDS_3           , MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_BINS_ALGWORDS_4           , MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_BINS_ALGWORDS_4           , MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_BINS_ALGWORDS_4           , MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_BINS_ALGWORDS_5           , MIL_IN_BINS1_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_BINS_ALGWORDS_5           , MIL_IN_BINS2_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_BINS_ALGWORDS_5           , MIL_IN_BINS3_SP2_SA2, E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_KUTR_G_RSUM               , MIL_OUT_KUTR_A_NVG, E_P_MIL_H4_L16_S0_M4096, 1);
    addParamToTable(P_KUTR_G_RSUM               , MIL_OUT_KUTR_R_OSO, E_P_MIL_H4_L16_S0_M4096, 1);
    addParamToTable(P_KUTR_G_RSUM               , AR_IN_BKS4_KUTR, E_P_AR_H28_L16_S0_M4096, 0100);
    addParamToTable(P_KUTR_G_RSUM               , AR_OUT_KUTR_BKS, E_P_AR_H28_L16_S0_M4096, 0100);
    addParamToTable(P_KUTR_Q                    , MIL_OUT_KUTR_A_NVG, E_P_MIL_H4_L16_S0_M163_84, 2);
    addParamToTable(P_KUTR_Q                    , MIL_OUT_KUTR_R_OSO, E_P_MIL_H4_L16_S0_M163_84, 2);
    addParamToTable(P_KUTR_Q                    , AR_IN_BKS4_KUTR, E_P_AR_H28_L16_S0_M163_84, 0101);
    addParamToTable(P_KUTR_Q                    , AR_OUT_KUTR_BKS, E_P_AR_H28_L16_S0_M163_84, 0101);
    addParamToTable(P_KUTR_M_Q                  , MIL_OUT_KUTR_A_NVG, E_P_MIL_H4_L16_S0_M2_4576, 3);
    addParamToTable(P_KUTR_M_Q                  , MIL_OUT_KUTR_R_OSO, E_P_AR_H28_L16_S0_M2_4576, 3);
    addParamToTable(P_KUTR_M_Q                  , AR_IN_BKS4_KUTR, E_P_AR_H28_L16_S0_M2_4576, 0102);
    addParamToTable(P_KUTR_M_Q                  , AR_OUT_KUTR_BKS, E_P_AR_H28_L16_S0_M2_4576, 0102);
    addParamToTable(P_KUTR_QPEREP           , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L9_S0_M32, 4);
    addParamToTable(P_KUTR_QPEREP           , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L9_S0_M32, 4);
    addParamToTable(P_KUTR_QPEREP           , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M32, 0103);
    addParamToTable(P_KUTR_QPEREP           , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M32, 0103);
    addParamToTable(P_KUTR_M_QPEREP         , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L9_S0_M0_48, 5);
    addParamToTable(P_KUTR_M_QPEREP         , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L9_S0_M0_48, 5);
    addParamToTable(P_KUTR_M_QPEREP         , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M32, 0104);
    addParamToTable(P_KUTR_M_QPEREP         , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M32, 0104);
    addParamToTable(P_KUTR_TTOP_PDK         , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H5_L12_S0_M128, 6);
    addParamToTable(P_KUTR_TTOP_PDK         , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H5_L12_S0_M128, 6);
    addParamToTable(P_KUTR_TTOP_PDK         , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M128, 0105);
    addParamToTable(P_KUTR_TTOP_PDK         , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M128, 0105);
    addParamToTable(P_KUTR_TTOP_PEREP       , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H5_L12_S0_M128, 7);
    addParamToTable(P_KUTR_TTOP_PEREP       , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H5_L12_S0_M128, 7);
    addParamToTable(P_KUTR_TTOP_PEREP       , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M128, 0106);
    addParamToTable(P_KUTR_TTOP_PEREP       , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M128, 0106);
    addParamToTable(P_KUTR_TTOP_B1          , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H5_L12_S0_M128, 8);
    addParamToTable(P_KUTR_TTOP_B1          , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H5_L12_S0_M128, 8);
    addParamToTable(P_KUTR_TTOP_B1          , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M128, 0110);
    addParamToTable(P_KUTR_TTOP_B1          , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M128, 0110);
    addParamToTable(P_KUTR_P1_TOP_B1        , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L13_S0_M0_512, 9);
    addParamToTable(P_KUTR_P1_TOP_B1        , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L13_S0_M0_512, 9);
    addParamToTable(P_KUTR_P1_TOP_B1        , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M128, 0111);
    addParamToTable(P_KUTR_P1_TOP_B1        , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M128, 0111);
    addParamToTable(P_KUTR_G_TSUM_IND       , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L14_S0_M1024, 10);
    addParamToTable(P_KUTR_G_TSUM_IND       , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L14_S0_M1024, 10);
    addParamToTable(P_KUTR_G_TSUM_IND       , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M1024, 0112);
    addParamToTable(P_KUTR_G_TSUM_IND       , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M1024, 0112);
    addParamToTable(P_KUTR_G_TB1            , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L14_S0_M1024, 11);
    addParamToTable(P_KUTR_G_TB1            , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L14_S0_M1024, 11);
    addParamToTable(P_KUTR_G_TB1            , AR_IN_BKS4_KUTR       , E_P_AR_H28_L16_S0_M1024, 0113);
    addParamToTable(P_KUTR_G_TB1            , AR_OUT_KUTR_BKS       , E_P_AR_H28_L16_S0_M1024, 0113);
    addParamToTable(P_KUTR_SRK1             , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_KUTR_SRK1             , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L19_S0_M0, 12);
    addParamToTable(P_KUTR_SRK1             , AR_IN_BKS4_KUTR       , E_P_MIL_H4_L19_S0_M0, 0130);
    addParamToTable(P_KUTR_SRK1             , AR_OUT_KUTR_BKS       , E_P_MIL_H4_L19_S0_M0, 0130);
    addParamToTable(P_KUTR_SRK2             , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_KUTR_SRK2             , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L19_S0_M0, 13);
    addParamToTable(P_KUTR_SRK2             , AR_IN_BKS4_KUTR       , E_P_MIL_H4_L19_S0_M0, 0131);
    addParamToTable(P_KUTR_SRK2             , AR_OUT_KUTR_BKS       , E_P_MIL_H4_L19_S0_M0, 0131);
    addParamToTable(P_KUTR_SRK3             , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_KUTR_SRK3             , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L19_S0_M0, 14);
    addParamToTable(P_KUTR_SRK3             , AR_IN_BKS4_KUTR       , E_P_MIL_H4_L19_S0_M0, 0132);
    addParamToTable(P_KUTR_SRK3             , AR_OUT_KUTR_BKS       , E_P_MIL_H4_L19_S0_M0, 0132);
    addParamToTable(P_KUTR_SRK4             , MIL_OUT_KUTR_A_NVG    , E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_KUTR_SRK4             , MIL_OUT_KUTR_R_OSO    , E_P_MIL_H4_L19_S0_M0, 15);
    addParamToTable(P_KUTR_SRK4             , AR_IN_BKS4_KUTR       , E_P_MIL_H4_L19_S0_M0, 0133);
    addParamToTable(P_KUTR_SRK4             , AR_OUT_KUTR_BKS       , E_P_MIL_H4_L19_S0_M0, 0133);

    addParamToTable(P_VIM_WORK_VOR_ILS		, -1,  TO_AR(14),  034,true);
    addParamToTable(P_VIM_FREQ_VOR_ILS_0_01_MHZ		, -1,  E_P_AR_H18_L15_S0_M0,  034);
    addParamToTable(P_VIM_FREQ_VOR_ILS_0_1_MHZ		, -1,  E_P_AR_H22_L19_S0_M0,  034);
    addParamToTable(P_VIM_FREQ_VOR_ILS_1_MHZ		, -1,  E_P_AR_H26_L23_S0_M0,  034);
    addParamToTable(P_VIM_FREQ_VOR_ILS_10_MHZ		, -1,  E_P_AR_H29_L27_S0_M0,  034);
    addParamToTable(P_VIM_MAT034		, -1,  E_P_AR_H31_L30_S0_M0,  034);
    addParamToTable(P_VIM_WORK_ILS_CP50		, -1,  TO_AR(13),  033,true);
    addParamToTable(P_VIM_FREQ_ILS_0_01_MHZ		, -1,  E_P_AR_H18_L15_S0_M0,  033);
    addParamToTable(P_VIM_FREQ_ILS_0_1_MHZ		, -1,  E_P_AR_H22_L19_S0_M0,  033);
    addParamToTable(P_VIM_FREQ_ILS_1_MHZ		, -1,  E_P_AR_H26_L23_S0_M0,  033);
    addParamToTable(P_VIM_FREQ_ILS_10_MHZ		, -1,  E_P_AR_H29_L27_S0_M0,  033);
    addParamToTable(P_VIM_MAT033		, -1,  E_P_AR_H31_L30_S0_M0,  033);
    addParamToTable(P_VIM_ZAP_PER1		, -1,  TO_AR(11),  0173,true);
    addParamToTable(P_VIM_EK		, -1,  E_P_AR_H28_L17_S1_M0_2,  0173);
    addParamToTable(P_BIM_MAT_EK		, -1,  E_P_AR_H31_L30_S0_M0,  0173);
    addParamToTable(P_VIM_ZAP_PER2		, -1,  TO_AR(11),  0174,true);
    addParamToTable(P_VIM_EG		, -1,  E_P_AR_H28_L17_S1_M0_4,  0174);
    addParamToTable(P_VIM_MAT_EG		, -1,  E_P_AR_H31_L30_S0_M0,  0174);
    addParamToTable(P_VIM_STS		, -1,  E_P_AR_H29_L9_S0_M0,  0350);
    addParamToTable(P_VIM_MAT_STS		, -1,  E_P_AR_H31_L30_S0_M0,  0350);
    addParamToTable(P_BKS2_BCVM_SDS_1_2		, -1,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(P_BKS2_BCVM_SDS_2_2		, -1,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(P_BKS2_BCVM_SDS_3_2		, -1,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(P_BKS2_BCVM_T3_PRB		, -1,  E_P_MIL_H4_L18_S1_M64,  3);
    addParamToTable(P_BKS2_BCVM_D_T3_PRB		, -1,  TO_MIL(19),  3,true);
    addParamToTable(P_BKS2_BCVM_T5_SP56		, -1,  E_P_MIL_H4_L18_S1_M64,  4);
    addParamToTable(P_BKS2_BCVM_D_T5_SP56		, -1,  TO_MIL(19),  4,true);
    addParamToTable(P_BKS2_BCVM_DD2_2		, -1,  E_P_MIL_H4_L18_S1_M0_125,  5);
    addParamToTable(P_BKS2_BCVM_D_DD2_2		, -1,  TO_MIL(19),  5,true);
    addParamToTable(P_BKS2_BCVM_DD3_2		, -1,  65,  6);
    addParamToTable(P_BKS2_BCVM_D_DD3_2		, -1,  TO_MIL(19),  6,true);
    addParamToTable(P_BKS2_BCVM_U_fazaA_right		, -1,  E_P_MIL_H5_L18_S0_M128,  2);
    addParamToTable(P_BKS2_BCVM_D_U_fazaA_right		, -1,  TO_MIL(19),  2,true);
    addParamToTable(P_BKS2_BCVM_U_fazaB_right		, -1,  E_P_MIL_H5_L18_S0_M128,  3);
    addParamToTable(P_BKS2_BCVM_D_U_fazaB_right		, -1,  TO_MIL(19),  3,true);
    addParamToTable(P_BKS2_BCVM_U_fazaC_right		, -1,  E_P_MIL_H5_L18_S0_M128,  4);
    addParamToTable(P_BKS2_BCVM_D_U_fazaC_right		, -1,  TO_MIL(19),  4,true);
    addParamToTable(P_BKS2_BCVM_f_RkanPer_I		, -1,  E_P_MIL_H5_L18_S0_M256,  5);
    addParamToTable(P_BKS2_BCVM_D_f_RkanPer_I		, -1,  TO_MIL(19),  5,true);
    addParamToTable(P_BKS2_BCVM_U_leftav_shin_2		, -1,  62,  6);
    addParamToTable(P_BKS2_BCVM_D_U_leftav_shin_2		, -1,  TO_MIL(19),  6,true);
    addParamToTable(P_BKS2_BCVM_U_rightav_shin_2		, -1,  E_P_MIL_H4_L18_S1_M32,  7);
    addParamToTable(P_BKS2_BCVM_D_U_rightav_shin_2		, -1,  TO_MIL(19),  7,true);
    addParamToTable(P_BKS2_BCVM_I_VU2		, -1,  E_P_MIL_H5_L18_S0_M256,  8);
    addParamToTable(P_BKS2_BCVM_D_I_VU2		, -1,  TO_MIL(19),  8,true);
    addParamToTable(P_BKS2_BCVM_I_VU3		, -1,  E_P_MIL_H5_L18_S0_M256,  9);
    addParamToTable(P_BKS2_BCVM_D_I_VU3		, -1,  TO_MIL(19),  9,true);
    addParamToTable(P_BKS2_BCVM_S_S_BKS_2		, -1,  E_P_MIL_H4_L19_S0_M0,  7);
    addParamToTable(P_BKS2_BCVM_K_NK_BKS2		, -1,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(P_BKS2_BCVM_MVI_SS_2		, -1,  E_P_MIL_H4_L19_S0_M0,  8);
    addParamToTable(P_BKS2_BCVM_MPA_SS_2		, -1,  E_P_MIL_H4_L19_S0_M0,  9);
    addParamToTable(P_BKS2_BCVM_MPA_SS1_2		, -1,  E_P_MIL_H4_L19_S0_M0,  10);
    addParamToTable(P_BKS2_BCVM_MPA_SS2_2		, -1,  E_P_MIL_H4_L19_S0_M0,  11);
    addParamToTable(P_BKS2_BCVM_MSK_SS_2		, -1,  E_P_MIL_H4_L19_S0_M0,  12);
    addParamToTable(P_BKS2_BCVM_Operating_Time_2		, -1,  E_P_MIL_H4_L19_S0_M0,  13);
    addParamToTable(P_BKS2_BCVM_SJS_BKS_2		, -1,  E_P_MIL_H4_L19_S0_M0,  14);
    addParamToTable(P_BKS2_BCVM_KS_2		, -1,  E_P_MIL_H4_L35_S0_M0,  15);
    addParamToTable(P_BKS2_BCVM_SDZU_2		, -1,  E_P_MIL_H4_L19_S0_M0,  17);
    addParamToTable(P_BKS2_BCVM_TIMESTART_2		, -1,  E_P_MIL_H4_L19_S0_M0,  18);
    addParamToTable(P_BKS2_BCVM_TIMESTOP_2		, -1,  E_P_MIL_H4_L19_S0_M0,  19);
    addParamToTable(P_BKS2_BCVM_SCB2		, -1,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(P_BKS2_BCVM_K_SCU_1_2		, -1,  E_P_MIL_H4_L19_S0_M0,  20);

    addParamToTable(BCVM_BKS2_SCU_1_2               , MIL_IN_BKS2_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(BCVM_BKS2_SCU_2_2               , MIL_IN_BKS2_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(BCVM_BKS2_SCU_4_2               , MIL_IN_BKS2_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(BCVM_BKS2_SCU_5_2               , MIL_IN_BKS2_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  4);

    addParamToTable(BCVM_BKS2_SCU_3_2               , MIL_IN_BKS2_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  1);

    addParamToTable(P_BKS3_BCVM_T7_EV4              , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L18_S1_M64,        1);
    addParamToTable(P_BKS3_BCVM_D_T7_EV4            , MIL_OUT_BKS3_NVG_SA1,  TO_MIL(19),                   1,true);
    addParamToTable(P_BKS3_BCVM_T7_EV4              , AR_OUT_BKS3_KSS,  E_P_AR_H29_L15_S1_M64,        0121);
    addParamToTable(P_BKS3_BCVM_D_T7_EV4            , AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0121);


    addParamToTable(P_BKS3_BCVM_DD1_3               , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L18_S1_M0_125,    2);
    addParamToTable(P_BKS3_BCVM_D_DD1_3             , MIL_OUT_BKS3_NVG_SA1,  TO_MIL(19),                   2,true);
    addParamToTable(P_BKS3_BCVM_DD1_3              , AR_OUT_BKS3_KSS,  E_P_AR_H29_L15_S1_M0_125,        0123);
    addParamToTable(P_BKS3_BCVM_D_DD1_3            , AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0123);

    addParamToTable(P_BKS3_BCVM_DD4_3               , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L18_S1_M0_125,    3);
    addParamToTable(P_BKS3_BCVM_D_DD4_3             , MIL_OUT_BKS3_NVG_SA1,  TO_MIL(19),                   3,true);
    addParamToTable(P_BKS3_BCVM_DD4_3              , AR_OUT_BKS3_KSS,  E_P_AR_H29_L15_S1_M0_125,        0124);
    addParamToTable(P_BKS3_BCVM_D_DD4_3            , AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0124);

    addParamToTable(P_BKS3_BCVM_SDS_1_3             , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  4);
    addParamToTable(P_BKS3_BCVM_SDS_2_3             , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  5);
    addParamToTable(P_BKS3_BCVM_S_S_BKS_3           , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  6);
    addParamToTable(P_BKS3_BCVM_MVI_SS_3            , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  7);
    addParamToTable(P_BKS3_BCVM_MPA_SS_3            , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  8);
    addParamToTable(P_BKS3_BCVM_MPA_SS1_3           , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  9);
    addParamToTable(P_BKS3_BCVM_MPA_SS2_3           , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  10);
    addParamToTable(P_BKS3_BCVM_MSK_SS_3            , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  11);
    addParamToTable(P_BKS3_BCVM_Operating_Time_3    , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  12);
    addParamToTable(P_BKS3_BCVM_SJS_BKS_3           , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  13);
    addParamToTable(P_BKS3_BCVM_KS_3                , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L35_S0_M0,  14);
    addParamToTable(P_BKS3_BCVM_SDZU_3              , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  16);
    addParamToTable(P_BKS3_BCVM_TIMESTART_3         , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  17);
    addParamToTable(P_BKS3_BCVM_TIMESTOP_3          , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  18);
    addParamToTable(P_BKS3_BCVM_SCB3                , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  6);
    addParamToTable(P_BKS3_BCVM_K_SCU_1_3           , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  19);
    addParamToTable(P_BKS3_BCVM_SDS_6_3             , MIL_OUT_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  20);


    addParamToTable(P_BKS3_BCVM_P_1GS       ,  MIL_OUT_BKS3_NVG_SA2,  E_P_MIL_H5_L18_S0_M256,   1);
    addParamToTable(P_BKS3_BCVM_D_P_1GS     ,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  1,true);
    addParamToTable(P_BKS3_BCVM_P_1GS       ,  AR_OUT_BKS3_KSS,  E_P_AR_H28_L15_S0_M256,        0106);
    addParamToTable(P_BKS3_BCVM_P_1GS       ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,        0106);
    addParamToTable(P_BKS3_BCVM_D_P_1GS     ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0106);

    addParamToTable(P_BKS3_BCVM_P_2GS       ,  MIL_OUT_BKS3_NVG_SA2,  E_P_MIL_H5_L18_S0_M256,  2);
    addParamToTable(P_BKS3_BCVM_D_P_2GS     ,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  2,true);

    addParamToTable(P_BKS3_BCVM_P_2GS       ,  AR_OUT_BKS3_KSS,  E_P_MIL_H5_L18_S0_M256,  0107);
    addParamToTable(P_BKS3_BCVM_P_2GS       ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,  0107);
    addParamToTable(P_BKS3_BCVM_D_P_2GS     ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0107);


    addParamToTable(P_BKS3_BCVM_P_GA_1GS    ,  MIL_OUT_BKS3_NVG_SA2,  E_P_MIL_H5_L18_S0_M256,  3);
    addParamToTable(P_BKS3_BCVM_P_GA_1GS    ,  AR_OUT_BKS3_KSS,  E_P_AR_H28_L15_S0_M256,  0103);
    addParamToTable(P_BKS3_BCVM_P_GA_1GS    ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,  0103);
    addParamToTable(P_BKS3_BCVM_D_P_GA_1GS  ,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  3,true);
    addParamToTable(P_BKS3_BCVM_D_P_GA_1GS  ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0103);

    addParamToTable(P_BKS3_BCVM_PerSht_1GS  ,  MIL_OUT_BKS3_NVG_SA2,  E_P_MIL_H5_L18_S0_M256,  4);
    addParamToTable(P_BKS3_BCVM_PerSht_1GS  ,  AR_OUT_BKS3_KSS,  E_P_AR_H28_L15_S0_M256,  0104);
    addParamToTable(P_BKS3_BCVM_PerSht_1GS  ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,  0104);
    addParamToTable(P_BKS3_BCVM_D_PerSht_1GS,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  4,true);
    addParamToTable(P_BKS3_BCVM_D_PerSht_1GS  ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0104);

    addParamToTable(P_BKS3_BCVM_PerSht_2GS  ,  MIL_OUT_BKS3_NVG_SA2, E_P_MIL_H5_L18_S0_M256 ,  5);
    addParamToTable(P_BKS3_BCVM_PerSht_2GS  ,  AR_OUT_BKS3_KSS,  E_P_AR_H28_L15_S0_M256,  0105);
    addParamToTable(P_BKS3_BCVM_PerSht_2GS  ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,  0105);
    addParamToTable(P_BKS3_BCVM_D_PerSht_2GS,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  5,true);
    addParamToTable(P_BKS3_BCVM_D_PerSht_2GS  ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0105);

    addParamToTable(P_BKS3_BCVM_P_pnev      ,  MIL_OUT_BKS3_NVG_SA2,  E_P_MIL_H5_L18_S0_M256,  6);
    addParamToTable(P_BKS3_BCVM_P_pnev      ,  AR_OUT_BKS3_KSS,  E_P_AR_H28_L15_S0_M256,  0112);
    addParamToTable(P_BKS3_BCVM_P_pnev      ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,  0112);
    addParamToTable(P_BKS3_BCVM_D_P_pnev    ,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  6,true);
    addParamToTable(P_BKS3_BCVM_D_P_pnev  ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0112);

    addParamToTable(P_BKS3_BCVM_Trj_1GS     ,  MIL_OUT_BKS3_NVG_SA2,  E_P_MIL_H5_L18_S0_M128,  7);
    addParamToTable(P_BKS3_BCVM_Trj_1GS     ,  AR_OUT_BKS3_KSS,  E_P_AR_H28_L15_S0_M256,  0110);
    addParamToTable(P_BKS3_BCVM_Trj_1GS      ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,  0110);
    addParamToTable(P_BKS3_BCVM_D_Trj_1GS   ,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  7,true);
    addParamToTable(P_BKS3_BCVM_D_Trj_1GS  ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0110);

    addParamToTable(P_BKS3_BCVM_Trj_2GS     ,  MIL_OUT_BKS3_NVG_SA2,  E_P_MIL_H5_L18_S0_M128,  8);
    addParamToTable(P_BKS3_BCVM_Trj_2GS     ,  AR_OUT_BKS3_KSS,  E_P_AR_H28_L15_S0_M256,  0111);
    addParamToTable(P_BKS3_BCVM_Trj_2GS     ,  AR_IN_IKRL_1_BKS3,  E_P_AR_H28_L15_S0_M256,  0111);
    addParamToTable(P_BKS3_BCVM_D_Trj_2GS   ,  MIL_OUT_BKS3_NVG_SA2,  TO_MIL(19),  8,true);
    addParamToTable(P_BKS3_BCVM_D_Trj_2GS  ,  AR_OUT_BKS3_KSS,  E_P_AR_H31_L30_S0_M0,  0111);

    addParamToTable(P_BKS3_BCVM_ObK_Pup_D   ,  MIL_OUT_BKS3_NVG_SA3,  E_P_MIL_H5_L18_S0_M64,  1);
    addParamToTable(P_BKS3_BCVM_D_ObK_Pup_D ,  MIL_OUT_BKS3_NVG_SA3,toMIL,19,   1 )    ;

    addParamToTable(P_BKS3_BCVM_Tgasturb_D  ,  MIL_OUT_BKS3_NVG_SA3,  E_P_MIL_H5_L18_S0_M1024,  2);
    addParamToTable(P_BKS3_BCVM_D_Tgasturb_D , MIL_OUT_BKS3_NVG_SA3,toMIL,19,   2 )    ;

    addParamToTable(P_BKS3_BCVM_SDS_4_3     , MIL_OUT_BKS3_NVG_SA3,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(P_BKS3_BCVM_SDS_5_3     , MIL_OUT_BKS3_NVG_SA3,  E_P_MIL_H4_L19_S0_M0,  4);

    addParamToTable(P_BKS3_BCVM_K_NK_BKS3   , MIL_OUT_BKS3_NVG_SA3,  E_P_MIL_H4_L19_S0_M0,  5);

    addParamToTable(BCVM_BKS3_SCU_1_3   , MIL_IN_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(BCVM_BKS3_SCU_2_3   , MIL_IN_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(BCVM_BKS3_SCU_4_3   , MIL_IN_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(BCVM_BKS3_SCU_5_3   , MIL_IN_BKS3_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  4);

    addParamToTable(BCVM_BKS3_SCU_3_3   , MIL_IN_BKS3_NVG_SA2,  E_P_MIL_H4_L19_S0_M0,  1);




    addParamToTable(P_BKS3_KSS_OUT1   , AR_OUT_BKS3_KSS,  E_P_AR_H31_L1_S0_M0,  0125);
    addParamToTable(P_BKS3_KSS_OUT2   , AR_OUT_BKS3_KSS,  E_P_AR_H31_L1_S0_M0,  0126);
    addParamToTable(P_BKS3_KSS_OUT3   , AR_OUT_BKS3_KSS,  E_P_AR_H31_L1_S0_M0,  0127);
    addParamToTable(P_BKS3_KSS_OUT4   , AR_OUT_BKS3_KSS,  E_P_AR_H31_L1_S0_M0,  0130);

    addParamToTable(P_BKS4_BCVM_T6_NChF   , MIL_OUT_BKS4_OSO_SA1,   E_P_MIL_H4_L18_S1_M64,  1);
    addParamToTable(P_BKS4_BCVM_D_T6_NChF , MIL_OUT_BKS4_OSO_SA1,   TO_MIL(19),  1,true);
    addParamToTable(P_BKS4_BCVM_DD2_4     , MIL_OUT_BKS4_OSO_SA1,   E_P_MIL_H4_L18_S1_M0_125,  2);
    addParamToTable(P_BKS4_BCVM_D_DD2_4   , MIL_OUT_BKS4_OSO_SA1,   TO_MIL(19),  2,true);
    addParamToTable(P_BKS4_BCVM_DD4_4     , MIL_OUT_BKS4_OSO_SA1,   E_P_MIL_H4_L18_S1_M0_125,  3);
    addParamToTable(P_BKS4_BCVM_D_DD4_4   , MIL_OUT_BKS4_OSO_SA1,   TO_MIL(19),  3,true);
    addParamToTable(P_BKS4_BCVM_G_Rsum    , MIL_OUT_BKS4_OSO_SA1,   E_P_MIL_H5_L18_S0_M4096,  4);
    addParamToTable(P_BKS4_BCVM_Q          , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H5_L18_S0_M128,  5);
    addParamToTable(P_BKS4_BCVM_M_Q        , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H5_L18_S0_M4,  6);
    addParamToTable(P_BKS4_BCVM_Qperep     , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H5_L18_S0_M32,  7);
    addParamToTable(P_BKS4_BCVM_M_Qperep   , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H5_L18_S0_M1,  8);
    addParamToTable(P_BKS4_BCVM_Ttop_pdk   , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L18_S1_M128,  9);
    addParamToTable(P_BKS4_BCVM_Ttop_perep , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L18_S1_M128,  10);
    addParamToTable(P_BKS4_BCVM_Ttop_B1    , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L18_S1_M64,  11);
    addParamToTable(P_BKS4_BCVM_Pl_top_B1  , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H5_L18_S0_M1,  12);
    addParamToTable(P_BKS4_BCVM_G_Tsum_ind , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H5_L18_S0_M1024,  13);
    addParamToTable(P_BKS4_BCVM_G_TB1      , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H5_L18_S0_M1024,  14);
    addParamToTable(P_BKS4_BCVM_SDS_1_4    , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  15);
    addParamToTable(P_BKS4_BCVM_SDS_2_4    , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  16);
    addParamToTable(P_BKS4_BCVM_SDS_3_4    , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  17);
    addParamToTable(P_BKS4_BCVM_SDS_4_4    , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  18);
    addParamToTable(P_BKS4_BCVM_SDS_5_4    , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  19);
    addParamToTable(P_BKS4_BCVM_SDS_7_4    , MIL_OUT_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  20);

    addParamToTable(P_BKS4_BCVM_S_S_BKS_4    ,  MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(P_BKS4_BCVM_K_NK_BKS4    ,  MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(P_BKS4_BCVM_MVI_SS_4     ,  MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(P_BKS4_BCVM_MPA_SS_4     ,  MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  4);
    addParamToTable(P_BKS4_BCVM_MPA_SS1_4    ,  MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  5);
    addParamToTable(P_BKS4_BCVM_MPA_SS2_4    ,  MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  6);
    addParamToTable(P_BKS4_BCVM_MSK_SS_4     ,  MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  7);
    addParamToTable(P_BKS4_BCVM_STS_BKS_4     , MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  8);
    addParamToTable(P_BKS4_BCVM_SKS_4         , MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L35_S0_M0,  9);
    addParamToTable(P_BKS4_BCVM_SDZU_4        , MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  11);
    addParamToTable(P_BKS4_BCVM_TIMESTART_4   , MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  12);
    addParamToTable(P_BKS4_BCVM_TIMESTOP_4    , MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  13);
    addParamToTable(P_BKS4_BCVM_K_SCU_1_4     , MIL_OUT_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  14);


    addParamToTable(P_BKS4_BCVM_P_1GS          ,  MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H5_L18_S0_M256,  1);
    addParamToTable(P_BKS4_BCVM_D_P_1GS        ,  MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  1,true);
    addParamToTable(P_BKS4_BCVM_P_2GS          ,  MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H5_L18_S0_M256,  2);
    addParamToTable(P_BKS4_BCVM_D_P_2GS        ,  MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  2,true);
    addParamToTable(P_BKS4_BCVM_P_GA_2GS       ,  MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H5_L18_S0_M256,  3);
    addParamToTable(P_BKS4_BCVM_D_P_GA_2GS     ,  MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  3,true);
    addParamToTable(P_BKS4_BCVM_PerSht_1GS     ,  MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H5_L18_S0_M256,  4);
    addParamToTable(P_BKS4_BCVM_D_PerSht_1GS    , MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  4,true);
    addParamToTable(P_BKS4_BCVM_PerSht_2GS      , MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H5_L18_S0_M256,  5);
    addParamToTable(P_BKS4_BCVM_D_PerSht_2GS    , MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  5,true);
    addParamToTable(P_BKS4_BCVM_P_pnev          , MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H5_L18_S0_M256,  6);
    addParamToTable(P_BKS4_BCVM_D_P_pnev        , MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  6,true);
    addParamToTable(P_BKS4_BCVM_Trj_1GS         , MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H4_L18_S1_M128,  7);
    addParamToTable(P_BKS4_BCVM_D_Trj_1GS       , MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  7,true);
    addParamToTable(P_BKS4_BCVM_Trj_2GS         , MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H4_L18_S1_M128,  8);
    addParamToTable(P_BKS4_BCVM_D_Trj_2GS       , MIL_OUT_BKS4_OSO_SA3,  TO_MIL(19),  8,true);
    addParamToTable(P_BKS4_BCVM_SDS_6_4         , MIL_OUT_BKS4_OSO_SA3,  E_P_MIL_H4_L19_S0_M0,  9);

    addParamToTable(P_BKS4_BCVM_K_NK_BKS4      , MIL_OUT_BKS4_OSO_SA4,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(P_BKS4_BCVM_SCB4           , MIL_OUT_BKS4_OSO_SA4,  E_P_MIL_H4_L19_S0_M0,  2);

    addParamToTable(BCVM_BKS4_SCU_1_4,          MIL_IN_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(BCVM_BKS4_SCU_3_4,          MIL_IN_BKS4_OSO_SA2,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(BCVM_BKS4_SCU_2_4,          MIL_IN_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(BCVM_BKS4_SCU_4_4,          MIL_IN_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(BCVM_BKS4_SCU_5_4,          MIL_IN_BKS4_OSO_SA1,  E_P_MIL_H4_L19_S0_M0,  4);


    addParamToTable(BKS1_BCVM_SDS_1_1           , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(BKS1_BCVM_SDS_2_1           , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(BKS1_BCVM_T1_SVO            , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L18_S1_M64,  3);
    addParamToTable(BKS1_BCVM_D_T1_SVO          , MIL_OUT_BKS1_NVG_SA1,  TO_MIL(19),  3,true);
    addParamToTable(BKS1_BCVM_T2_LB             , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L18_S1_M64,  4);
    addParamToTable(BKS1_BCVM_D_T2_LB           , MIL_OUT_BKS1_NVG_SA1,  TO_MIL(19),  4,true);
    addParamToTable(BKS1_BCVM_T4_SES            , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L18_S1_M64,  5);
    addParamToTable(BKS1_BCVM_D_T4_SES          , MIL_OUT_BKS1_NVG_SA1,  TO_MIL(19),  5,true);
    addParamToTable(BKS1_BCVM_DD1_1             , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L18_S1_M0_125,  6);
    addParamToTable(BKS1_BCVM_D_DD1_1           , MIL_OUT_BKS1_NVG_SA1,  TO_MIL(19),  6,true);
    addParamToTable(BKS1_BCVM_DD3_1             , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L18_S1_M0_125,  7);
    addParamToTable(BKS1_BCVM_D_DD3_1           , MIL_OUT_BKS1_NVG_SA1,  TO_MIL(19),  7,true);
    addParamToTable(BKS1_BCVM_S_S_BKS_1         , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  8);
    addParamToTable(BKS1_BCVM_MVI_SS_1          , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  9);
    addParamToTable(BKS1_BCVM_MPA_SS_1          , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  10);
    addParamToTable(BKS1_BCVM_MPA_SS1_1         , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  11);
    addParamToTable(BKS1_BCVM_MPA_SS2_1         , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  12);
    addParamToTable(BKS1_BCVM_MSK_SS_1          , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  13);
    addParamToTable(BKS1_BCVM_Operating_Time_1  , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  14);
    addParamToTable(BKS1_BCVM_SJS_BKS_1         , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  15);
    addParamToTable(BKS1_BCVM_KS_1              , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L35_S0_M0,  16);
    addParamToTable(BKS1_BCVM_SDZU_1            , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  18);
    addParamToTable(BKS1_BCVM_TIMER_START_1     , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  19);
    addParamToTable(BKS1_BCVM_TIMER_STOP_1      , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  20);
    addParamToTable(BKS1_BCVM_K_SCU_1_1         , MIL_OUT_BKS1_NVG_SA1,  E_P_MIL_H4_L19_S0_M0,  21);


    addParamToTable(BKS1_BCVM_SDS_3_1                , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H4_L19_S0_M0,    1);
    addParamToTable(BKS1_BCVM_U_fazaA_left           , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M128,  2);
    addParamToTable(BKS1_BCVM_D_U_fazaA_left         , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  2,true);
    addParamToTable(BKS1_BCVM_U_fazaB_left           , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M128,  3);
    addParamToTable(BKS1_BCVM_D_U_fazaB_left         , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  3,true);
    addParamToTable(BKS1_BCVM_U_fazaC_left           , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M128,  4);
    addParamToTable(BKS1_BCVM_D_U_fazaC_left         , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  4,true);
    addParamToTable(BKS1_BCVM_f_LkanPer_I            , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M256,  5);
    addParamToTable(BKS1_BCVM_D_f_LkanPer_I          , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  5,true);
    addParamToTable(BKS1_BCVM_U_osn_shin             , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M32,   6);
    addParamToTable(BKS1_BCVM_D_U_osn_shin           , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  6,true);
    addParamToTable(BKS1_BCVM_U_leftav_shin_1        , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M32,   7);
    addParamToTable(BKS1_BCVM_D_U_leftav_shin_1      , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  7,true);
    addParamToTable(BKS1_BCVM_U_rightav_shin_1       , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M32,   8);
    addParamToTable(BKS1_BCVM_D_U_rightav_shin_1     , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  8,true);
    addParamToTable(BKS1_BCVM_I_VU1                  , MIL_OUT_BKS1_NVG_SA2,  E_P_MIL_H5_L18_S0_M256,  9);
    addParamToTable(BKS1_BCVM_D_I_VU1                , MIL_OUT_BKS1_NVG_SA2,  TO_MIL(19),  9,true);

    addParamToTable(BKS1_BCVM_SDS_4_1     , MIL_OUT_BKS1_NVG_SA3,  E_P_MIL_H4_L19_S0_M0,  1);
    addParamToTable(BKS1_BCVM_K_NK_BKS1   , MIL_OUT_BKS1_NVG_SA3,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(BKS1_BCVM_SCB_1       , MIL_OUT_BKS1_NVG_SA3,  E_P_MIL_H4_L19_S0_M0,  3);

    addParamToTable(BCVM_BKS1_SCU_1_1,   MIL_IN_BKS1_NVG_SA1,E_P_MIL_H4_L19_S0_M0,1 );
    addParamToTable(BCVM_BKS1_SCU_2_1,   MIL_IN_BKS1_NVG_SA1,E_P_MIL_H4_L19_S0_M0,2 );
    addParamToTable(BCVM_BKS1_SCU_4_1,   MIL_IN_BKS1_NVG_SA1,E_P_MIL_H4_L19_S0_M0,3 );
    addParamToTable(BCVM_BKS1_SCU_5_1,   MIL_IN_BKS1_NVG_SA1,E_P_MIL_H4_L19_S0_M0,4 );

    addParamToTable(BCVM_BKS1_SCU_3_1,   MIL_IN_BKS1_NVG_SA2,E_P_MIL_H4_L19_S0_M0,1 );

    addParamToTable(P_BINS_SD1_B1_MC1		, -1,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(P_BINS_SD4_B1_MC1		, -1,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(P_BINS_SD3_B1_MC1		, -1,  E_P_MIL_H4_L19_S0_M0,  3);
    addParamToTable(P_BINS_SD2_B1_MC1		, -1,  E_P_MIL_H4_L19_S0_M0,  2);
    addParamToTable(P_BINS_SD5_B1_MC1		, -1,  E_P_MIL_H4_L19_S0_M0,  2);

    addParamToTable(P_APDD_NAV_NUM_SP_S_S            , -1,  E_P_AR_H29_L1_S0_M0,   060);
    addParamToTable(P_APDD_NAV_NUM_SP_S_S_MAT        , -1,  E_P_AR_H31_L30_S0_M0,  060);

    addParamToTable(P_APDD_NAV_MSL        , -1,  E_P_AR_H29_L9_S1_M65536,  076);
    addParamToTable(P_APDD_NAV_MSL_MAT        , -1,  E_P_AR_H31_L30_S0_M0,  076);
    addParamToTable(P_APDD_NAV_HDOP        , -1,  E_P_AR_H29_L14_S1_M512,  0101);
    addParamToTable(P_APDD_NAV_HDOP_MAT     , -1,  E_P_AR_H31_L30_S0_M0,  0101);
    addParamToTable(P_APDD_NAV_VDOP		, -1,  E_P_AR_H29_L14_S1_M512,  0102);
    addParamToTable(P_APDD_NAV_VDOP_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0102);
    addParamToTable(P_APDD_NAV_PUT_UGOL_IST		, -1,  E_P_AR_H29_L11_S1_M90,  0103);
    addParamToTable(P_APDD_NAV_PUT_UGOL_IST_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0103);
    addParamToTable(P_APDD_NAV_FI_SENL		, -1,  E_P_AR_H29_L9_S1_M90,  0110);
    addParamToTable(P_APDD_NAV_FI_SENL_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0110);
    addParamToTable(P_APDD_NAV_LAM_SENL		, -1,  E_P_AR_H29_L9_S1_M90,  0111);
    addParamToTable(P_APDD_NAV_LAM_SENL_MAT	, -1,  E_P_AR_H31_L30_S0_M0,  0111);
    addParamToTable(P_APDD_NAV_W		, -1,  E_P_AR_H29_L11_S1_M2048,  0112);
    addParamToTable(P_APDD_NAV_W_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0112);
    addParamToTable(P_APDD_NAV_FI		, -1,  E_P_AR_H29_L18_S1_M0_0000858,  0120);
    addParamToTable(P_APDD_NAV_FI_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0120);
    addParamToTable(P_APDD_NAV_LAM		, -1,  E_P_AR_H29_L18_S1_M0_0000858,  0121);
    addParamToTable(P_APDD_NAV_LAM_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0121);
    addParamToTable(P_APDD_NAV_HIL		, -1,  E_P_AR_H29_L12_S1_M8,  0130);
    addParamToTable(P_APDD_NAV_HIL_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0130);
    addParamToTable(P_APDD_NAV_VIL		, -1,  E_P_AR_H29_L11_S1_M16384,  0133);
    addParamToTable(P_APDD_NAV_VIL_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0133);
    addParamToTable(P_APDD_NAV_VFOM		, -1,  E_P_AR_H29_L11_S1_M16384,  0136);
    addParamToTable(P_APDD_NAV_VFOM_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0136);
    addParamToTable(P_APDD_NAV_VERT_V		, -1,  E_P_AR_H29_L11_S1_M16384,  0165);
    addParamToTable(P_APDD_NAV_VERT_V_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0165);
    addParamToTable(P_APDD_NAV_PR_W_S_U		, -1,  E_P_AR_H29_L11_S1_M2048,  0166);
    addParamToTable(P_APDD_NAV_PR_W_S_U_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0166);
    addParamToTable(P_APDD_NAV_PR_W_Z_W		, -1,  E_P_AR_H29_L11_S1_M2048,  0174);
    addParamToTable(P_APDD_NAV_PR_W_Z_W_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0174);
    addParamToTable(P_APDD_NAV_HFOM		, -1,  E_P_AR_H29_L11_S1_M8,  0247);
    addParamToTable(P_APDD_NAV_HFOM_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0247);
    addParamToTable(P_APDD_NAV_PRM_GNSS		, -1,  E_P_AR_H29_L11_S0_M0,  0273);
    addParamToTable(P_APDD_NAV_PRM_GNSS_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0273);
    addParamToTable(P_APDD_NAV_GNSS_OTKAZ		, -1,  E_P_AR_H29_L11_S0_M0,  0355);
    addParamToTable(P_APDD_NAV_GNSS_OTKAZ_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0355);
    addParamToTable(P_APDD_NAV_H_WGS_84		, -1,  E_P_AR_H29_L9_S1_M65536,  0370);
    addParamToTable(P_APDD_NAV_H_WGS_84_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0370);
    addParamToTable(P_APDD_PSI_VPP_180		, -1,  E_P_AR_H31_L1_S0_M0,  0105);
    addParamToTable(P_APDD_PSI_VPP_180_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  0105);
    addParamToTable(P_APDD_PSI_VPP_360		, -1,  E_P_AR_H29_L9_S0_M0,  017);
    addParamToTable(P_APDD_PSI_VPP_360_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  017);
    addParamToTable(P_APDD_PSI_VPP_360_MAT		, -1,  E_P_AR_H29_L9_S0_M0,  017);
    addParamToTable(P_APDD_PSI_VPP_360_MAT		, -1,  E_P_AR_H31_L30_S0_M0,  017);
    addParamToTable(P_APDD_LAND_EK_FUT              , -1,  E_P_AR_H29_L11_S1_M11993_088,  0116);
    addParamToTable(P_APDD_LAND_EK_FUT_MAT          , -1,  E_P_AR_H31_L30_S0_M0,  0116);
    addParamToTable(P_APDD_LAND_EG_FUT              , -1,  E_P_AR_H29_L15_S1_M512,  0117);
    addParamToTable(P_APDD_LAND_EG_FUT_MAT          , -1,  E_P_AR_H31_L30_S0_M0,  0117);
    addParamToTable(P_APDD_LAND_N_ISKL_SP_1         , -1,  E_P_AR_H15_L9_S0_M0,  0126);
    addParamToTable(P_APDD_LAND_N_ISKL_SP_1_MAT     , -1,  E_P_AR_H31_L30_S0_M0,  0126);
    addParamToTable(P_APDD_LAND_N_ISKL_SP_2         , -1,  E_P_AR_H15_L9_S0_M0,  0127);
    addParamToTable(P_APDD_LAND_N_ISKL_SP_2_MAT     , -1,  E_P_AR_H31_L30_S0_M0,  0127);
    addParamToTable(P_APDD_LAND_FOM_VERT            , -1,  E_P_AR_H28_L11_S0_M16384,  0142);
    addParamToTable(P_APDD_LAND_FOM_VERT_MAT        , -1,  E_P_AR_H31_L30_S0_M0,  0142);
    addParamToTable(P_APDD_LAND_FOM_GOR             , -1,  E_P_AR_H29_L11_S1_M2048,  0145);
    addParamToTable(P_APDD_LAND_FOM_GOR_MAT         , -1,  E_P_AR_H31_L30_S0_M0,  0145);
    addParamToTable(P_APDD_LAND_ZAP_EK              , -1,  TO_AR(11),  0173,true);
    addParamToTable(P_APDD_LAND_ILS_GLS_EK          , -1,  TO_AR(12),  0173,true);
    addParamToTable(P_APDD_LAND_EK_RGM              , -1,  E_P_AR_H29_L17_S1_M0_2,  0173);
    addParamToTable(P_APDD_LAND_EK_MAT              , -1,  E_P_AR_H31_L30_S0_M0,  0173);
    addParamToTable(P_APDD_LAND_ZAP_EG              , -1,  TO_AR(11),  0174,true);
    addParamToTable(P_APDD_LAND_ILS_GLS_EG          , -1,  TO_AR(12),  0174,true);
    addParamToTable(P_APDD_LAND_EG_RGM              , -1,  E_P_AR_H29_L17_S1_M0_4,  0174);
    addParamToTable(P_APDD_LAND_EG_MAT              , -1,  E_P_AR_H31_L30_S0_M0,  0174);
    addParamToTable(P_APDD_LAND_HIL                 , -1,  E_P_AR_H28_L11_S1_M7_86432,  0143);
    addParamToTable(P_APDD_LAND_HIL_MAT             , -1,  E_P_AR_H31_L30_S0_M0,  0143);
    addParamToTable(P_APDD_LAND_VIL                 , -1,  E_P_AR_H29_L11_S1_M16384,  0144);
    addParamToTable(P_APDD_LAND_VIL_MAT             , -1,  E_P_AR_H31_L30_S0_M0,  0144);
    addParamToTable(P_APDD_LAND_D_VPP_GOR           , -1,  E_P_AR_H29_L13_S1_M256,  0177);
    addParamToTable(P_APDD_LAND_D_VPP_GOR_MAT       , -1,  E_P_AR_H31_L30_S0_M0,  0177);
    addParamToTable(P_APDD_LAND_SS                  , -1,  E_P_AR_H29_L0_S0_M0,  0353);
    addParamToTable(P_APDD_LAND_SS_MAT              , -1,  E_P_AR_H31_L30_S0_M0,  0353);


    addParamToTable(P_APDD_PSI_VPP_180_IN           , -1,  E_P_AR_H31_L1_S0_M0,  0105);
    addParamToTable(P_APDD_SEL_REGIME               , -1,  E_P_AR_H14_L13_S0_M0,  033);
    addParamToTable(P_APDD_CH_GLS_HI                , -1,  E_P_AR_H12_L11_S0_M0,  033);
    addParamToTable(P_APDD_CH_GLS_LOW               , -1,  E_P_AR_H29_L15_S0_M0,  033);
    addParamToTable(P_APDD_MAT_033                  , -1,  E_P_AR_H31_L30_S0_M0,  033);

   addParamToTable(CIMSS_A_CISPR    , -1, E_P_MIL_H4_L18_S0_M0 ,1    );
   addParamToTable(CIMSS_A_NUM_PP   , -1, E_P_MIL_H4_L35_S0_M0 ,2    );
   addParamToTable(CIMSS_A_CNT_OBMEN, -1, E_P_MIL_H4_L11_S0_M0 ,4    );

   addParamToTable(CIMSS_A_SOST1    , -1, E_P_MIL_H4_L19_S0_M0 ,5    );
   addParamToTable(CIMSS_A_SOST2    , -1, E_P_MIL_H4_L19_S0_M0 ,6    );
   addParamToTable(CIMSS_A_SOST3    , -1, E_P_MIL_H4_L19_S0_M0 ,7    );
   addParamToTable(CIMSS_A_SOST4    , -1, E_P_MIL_H4_L19_S0_M0 ,8    );
   addParamToTable(CIMSS_A_SOST5    , -1, E_P_MIL_H4_L19_S0_M0 ,9    );
   addParamToTable(CIMSS_A_SOST6    , -1, E_P_MIL_H4_L19_S0_M0 ,10   );
   addParamToTable(CIMSS_A_SOST7    , -1, E_P_MIL_H4_L19_S0_M0 ,11   );
   addParamToTable(CIMSS_A_SOST8    , -1, E_P_MIL_H4_L19_S0_M0 ,12   );
   addParamToTable(CIMSS_A_SOST9    , -1, E_P_MIL_H4_L19_S0_M0 ,13   );



   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_12, -1,  E_P_AR_H29_L15_S1_M0_125,  0113);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_13, -1,  E_P_AR_H29_L15_S1_M0_125,  0114);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_20, -1,  E_P_AR_H29_L15_S1_M64,  0123);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_21, -1,  E_P_AR_H29_L15_S1_M64,  0124);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_22, -1,  E_P_AR_H29_L15_S1_M64,  0125);
   addParamToTable(P_IKRL1_BKS1_SDSBKS1_KSS_Out_2, -1,   E_P_AR_H29_L9_S0_M0,  0127);
   addParamToTable(P_IKRL1_BKS1_SDSBKS1_KSS_Out_3, -1,   E_P_AR_H29_L9_S0_M0,  0130);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_4, -1,   E_P_AR_H29_L9_S0_M0,  0103);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_5, -1,   E_P_AR_H29_L9_S0_M0,  0104);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_6, -1,   E_P_AR_H29_L9_S0_M0,  0105);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_7, -1,   E_P_AR_H29_L9_S0_M0,  0106);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_8, -1,   E_P_AR_H29_L9_S0_M0,  0107);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_9, -1,   E_P_AR_H29_L9_S0_M0,  0110);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_10, -1,   E_P_AR_H29_L9_S0_M0,  0111);
   addParamToTable(P_IKRL1_BKS1_SASBKS1_KSS_Out_11, -1,   E_P_AR_H29_L9_S0_M0,  0112);

   addParamToTable(P_IKRL1_BKS3_SASBKS3_KSS_Out_18, -1,   E_P_AR_H29_L9_S0_M0,  0121);
   addParamToTable(P_IKRL1_BKS3_SASBKS3_KSS_Out_20, -1,   E_P_AR_H29_L9_S0_M0,  0123);
   addParamToTable(P_IKRL1_BKS3_SASBKS3_KSS_Out_21, -1,   E_P_AR_H29_L9_S0_M0,  0124);
   addParamToTable(P_IKRL1_BKS3_SDSBKS3_KSS_Out_3, -1,   E_P_AR_H29_L9_S0_M0,  0127);
   addParamToTable(P_IKRL1_BKS3_SDSBKS3_KSS_Out_4, -1,   E_P_AR_H29_L9_S0_M0,  0130);


   addParamToTable(P_IKRL1_BCVM_SINVG_Out_1  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  01)    ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_2  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  02)    ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_3  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  03)    ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_4  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  04)    ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_5  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  05)    ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_6  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  06)    ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_7  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  07)    ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_8  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  010)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_9  ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  011)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_10 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  012)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_11 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  013)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_12 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  014)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_13 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  015)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_14 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  016)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_15 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  017)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_16 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  020)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_17 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  021)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_18 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  022)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_19 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  023)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_20 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  024)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_21 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  025)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_22 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  026)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_23 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  027)   ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_37 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  0134)  ;
   addParamToTable(P_IKRL1_BCVM_SINVG_Out_38 ,AR_IN_IKRL_1_BCVM, E_P_AR_H29_L9_S0_M0,  0135)  ;
   addParamToTable(P_SAU117_N2F,        MIL_OUT_BSS_OSO_D, E_P_MIL_H5_L18_S0_M64, 1);
   addParamToTable(P_SAU117_N2F,        AR_OUT_BSS117_S_BCVM_2,   E_P_AR_H28_L9_S0_M64, 0121);
   addParamToTable(P_SAU117_D_N2F,      MIL_OUT_BSS_OSO_D,TO_MIL(19),1,true);
   addParamToTable(P_SAU117_D_N2F,      AR_OUT_BSS117_S_BCVM_2,E_P_AR_H31_L30_S0_M0,0121);

   addParamToTable(P_SAU117_N2F,        MIL_OUT_BSS_OSO_S, E_P_MIL_H5_L18_S0_M64, 1);
   //addParamToTable(P_SAU117_N2F,        AR_OUT_BSS117_S_BCVM_2,   E_P_AR_H28_L9_S0_M64, 0121);
   addParamToTable(P_SAU117_D_N2F,      MIL_OUT_BSS_OSO_S,TO_MIL(19),1,true);
   //addParamToTable(P_SAU117_D_N2F,      AR_OUT_BSS117_S_BCVM_2,E_P_AR_H31_L30_S0_M0,0121);


   addParamToTable(P_SAU117_N2F,        MIL_OUT_ECR_L_58_S1_NVG, E_P_MIL_H5_L18_S0_M64, 1);
   addParamToTable(P_SAU117_D_N2F,      MIL_OUT_ECR_L_58_S1_NVG,TO_MIL(19),1,true);

   addParamToTable(P_SAU117_N2F,        MIL_OUT_ECR_L_58_S1_BP, E_P_MIL_H5_L18_S0_M64, 1);
   addParamToTable(P_SAU117_D_N2F,      MIL_OUT_ECR_L_58_S1_BP,TO_MIL(19),1,true);

   addParamToTable(P_SAU117_N2F,        MIL_OUT_ECR_R_58_S1_NVG, E_P_MIL_H5_L18_S0_M64, 1);
   addParamToTable(P_SAU117_D_N2F,      MIL_OUT_ECR_R_58_S1_NVG,TO_MIL(19),1,true);

   addParamToTable(P_SAU117_N2F,        MIL_OUT_ECR_R_58_S1_BP, E_P_MIL_H5_L18_S0_M64, 1);
   addParamToTable(P_SAU117_D_N2F,      MIL_OUT_ECR_R_58_S1_BP,TO_MIL(19),1,true);


   //addParamToTable(P_SAU117_STEP,       MIL_OUT_BSS_OSO_S, E_P_MIL_H7_L14_S0_M0, 6);
   //addParamToTable(P_SAU117_STEP,       MIL_OUT_BSS_OSO_D, E_P_MIL_H7_L14_S0_M0, 6);

   addParamToTable(P_SAU117_T4COR,      MIL_OUT_BSS_OSO_D, E_P_MIL_H5_L18_S0_M512, 2);
   addParamToTable(P_SAU117_D_T4,       MIL_OUT_BSS_OSO_D, TO_MIL(19),2,true);
   addParamToTable(P_SAU117_T4COR,      MIL_OUT_ECR_L_58_S1_NVG, E_P_MIL_H5_L18_S0_M512, 2);
   addParamToTable(P_SAU117_D_T4,       MIL_OUT_ECR_L_58_S1_NVG, TO_MIL(19),2,true);
   addParamToTable(P_SAU117_T4COR,      MIL_OUT_ECR_L_58_S1_BP, E_P_MIL_H5_L18_S0_M512, 2);
   addParamToTable(P_SAU117_D_T4,       MIL_OUT_ECR_L_58_S1_BP, TO_MIL(19),2,true);

   addParamToTable(P_SAU117_T4COR,      MIL_OUT_ECR_R_58_S1_NVG, E_P_MIL_H5_L18_S0_M512, 2);
   addParamToTable(P_SAU117_D_T4,       MIL_OUT_ECR_R_58_S1_NVG, TO_MIL(19),2,true);
   addParamToTable(P_SAU117_T4COR,      MIL_OUT_ECR_R_58_S1_BP, E_P_MIL_H5_L18_S0_M512, 2);
   addParamToTable(P_SAU117_D_T4,       MIL_OUT_ECR_R_58_S1_BP, TO_MIL(19),2,true);


   addParamToTable(P_SAU117_T4COR,      AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L9_S0_M512, 0122);
   addParamToTable(P_SAU117_D_T4,       AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0122);

   addParamToTable(P_SAU117_T4COR_MAX,  MIL_OUT_BSS_OSO_D, E_P_MIL_H5_L19_S0_M512, 3);
   addParamToTable(P_SAU117_T4COR_MAX,  MIL_OUT_BSS_OSO_S, E_P_MIL_H5_L19_S0_M512, 3);

   addParamToTable(P_SAU117_T4COR_MAX,  AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L9_S0_M512, 0123);
   addParamToTable(P_SAU117_D_T4COR_MAX,AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0123);

   addParamToTable(P_SAU117_STEP,  AR_OUT_BSS117_S_BCVM_2, E_P_AR_H24_L9_S0_M0, 0124);
   addParamToTable(P_SAU117_D_STEP,AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0124);

   addParamToTable(P_SAU117_SDS_1,  AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L1_S0_M0, 0126);
   addParamToTable(P_SAU117_D_SDS_1,AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0126);


   addParamToTable(P_SAU117_SDS_2,  AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L1_S0_M0, 0127);
   addParamToTable(P_SAU117_D_SDS_2,AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0127);

   addParamToTable(P_SAU117_SDS_3,  AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L1_S0_M0, 0130);
   addParamToTable(P_SAU117_D_SDS_3,AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0130);




   addParamToTable(P_SAU117_T4COR,      MIL_OUT_BSS_OSO_S, E_P_MIL_H5_L18_S0_M512, 2);
   addParamToTable(P_SAU117_D_T4,       MIL_OUT_BSS_OSO_S,TO_MIL(19),2,true);

   addParamToTable(P_SAU117_SDS_1,      MIL_OUT_BSS_OSO_S, E_P_MIL_H4_L19_S0_M0, 4);
   addParamToTable(P_SAU117_SDS_1,      MIL_OUT_BSS_OSO_D, E_P_MIL_H4_L19_S0_M0, 4);

   addParamToTable(P_SAU117_SDS_2,      MIL_OUT_BSS_OSO_S, E_P_MIL_H4_L19_S0_M0, 5);
   addParamToTable(P_SAU117_SDS_2,      MIL_OUT_BSS_OSO_D, E_P_MIL_H4_L19_S0_M0, 5);

   addParamToTable(P_SAU117_SDS_3,      MIL_OUT_BSS_OSO_S, E_P_MIL_H4_L19_S0_M0, 6);
   addParamToTable(P_SAU117_SDS_3,      MIL_OUT_BSS_OSO_D, E_P_MIL_H4_L19_S0_M0, 6);

   addParamToTable(P_SAU117_N1F,        MIL_OUT_BSS_OSO_D, E_P_MIL_H5_L18_S0_M64, 7);
   addParamToTable(P_SAU117_D_N1,       MIL_OUT_BSS_OSO_D,TO_MIL(19),7,true);

   addParamToTable(P_SAU117_N1F,        MIL_OUT_BSS_OSO_S, E_P_MIL_H5_L18_S0_M64, 7);
   addParamToTable(P_SAU117_D_N1,       MIL_OUT_BSS_OSO_S,TO_MIL(19),7,true);

   addParamToTable(P_SAU117_N1F,        AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L19_S0_M64, 0131);
   addParamToTable(P_SAU117_D_N1,       AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0131);

   addParamToTable(P_SAU117_T1F,        MIL_OUT_BSS_OSO_D, E_P_MIL_H5_L18_S0_M256, 8);
   addParamToTable(P_SAU117_D_T1,       MIL_OUT_BSS_OSO_D,TO_MIL(19),8,true);

   addParamToTable(P_SAU117_T1F,        AR_OUT_BSS117_S_BCVM_2, E_P_AR_H29_L19_S1_M256, 0132);
   addParamToTable(P_SAU117_D_T1,       AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0, 0132);



   addParamToTable(P_SAU117_T1F,        MIL_OUT_BSS_OSO_S, E_P_MIL_H5_L18_S0_M256, 8);
   addParamToTable(P_SAU117_D_T1,       MIL_OUT_BSS_OSO_S,TO_MIL(19),8,true);

   addParamToTable(P_SAU117_SDS_4,      MIL_OUT_BSS_OSO_S, E_P_MIL_H4_L19_S0_M0, 9);
   addParamToTable(P_SAU117_SDS_4,      MIL_OUT_BSS_OSO_D, E_P_MIL_H4_L19_S0_M0, 9);

   addParamToTable(P_SAU117_R_PROG,     MIL_OUT_BSS_OSO_S, E_P_MIL_H5_L19_S0_M128, 10);
   addParamToTable(P_SAU117_R_PROG,     MIL_OUT_BSS_OSO_D, E_P_MIL_H5_L19_S0_M128, 10);

   addParamToTable(P_SAU117_R_PROG,     AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L19_S0_M128, 0134);
   addParamToTable(P_SAU117_D_R_PROG,   AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0,   0134);



   addParamToTable(P_SAU117_RF,         MIL_OUT_BSS_OSO_D, E_P_MIL_H5_L18_S0_M128, 11);
   addParamToTable(P_SAU117_D_RF,       MIL_OUT_BSS_OSO_D,TO_MIL(19),11,true);

   addParamToTable(P_SAU117_RF,         MIL_OUT_BSS_OSO_S, E_P_MIL_H5_L18_S0_M128, 11);
   addParamToTable(P_SAU117_D_RF,       MIL_OUT_BSS_OSO_S,TO_MIL(19),11,true);

   addParamToTable(P_SAU117_RF,         AR_OUT_BSS117_S_BCVM_2, E_P_AR_H28_L19_S0_M128, 0135);
   addParamToTable(P_SAU117_D_RF,       AR_OUT_BSS117_S_BCVM_2, E_P_AR_H31_L30_S0_M0,   0135);
  addParamToTable(P_COUNT_PARAM,   AR_OUT_VSU_UCOKS, E_P_AR_H31_L9_S0_M0,   021);
   addParamToTable(P_COUNT_PARAM_M,   AR_OUT_VSU_UCOKS, E_P_AR_H31_L30_S0_M0,   021);


   //! ЛЛ БКС1
   addParamToTable(P_BKS1_58_BCVM_SDS_1_1             , MIL_OUT_BKS1_58_IUS_SA1,  E_P_MIL_H4_L19_S0_M0,  11);
   addParamToTable(P_BKS1_58_BCVM_SDS_3_1             , MIL_OUT_BKS1_58_IUS_SA1,  E_P_MIL_H4_L19_S0_M0,  12);
   addParamToTable(P_BKS1_58_BCVM_SDS_4_1             , MIL_OUT_BKS1_58_IUS_SA1,  E_P_MIL_H4_L19_S0_M0,  13);
   addParamToTable(P_BKS1_58_BCVM_SDS_22_1            , MIL_OUT_BKS1_58_IUS_SA1,  E_P_MIL_H4_L19_S0_M0,  14);
   addParamToTable(P_BKS1_58_BCVM_SDS_23_1            , MIL_OUT_BKS1_58_IUS_SA1,  E_P_MIL_H4_L19_S0_M0,  18);
   addParamToTable(P_BKS1_58_BCVM_Operating_Time_1    , MIL_OUT_BKS1_58_IUS_SA1,  E_P_MIL_H4_L19_S0_M0,  15);
   addParamToTable(P_BKS1_58_BCVM_SJS_BKS_1           , MIL_OUT_BKS1_58_IUS_SA1,  E_P_MIL_H4_L19_S0_M0,  16);

   addParamToTable(P_BKS1_58_BCVM_SDS_2_1           , MIL_OUT_BKS1_58_IUS_SA2,  E_P_MIL_H4_L19_S0_M0,  11);
   addParamToTable(P_BKS1_58_BCVM_SDS_5_1           , MIL_OUT_BKS1_58_IUS_SA2,  E_P_MIL_H4_L19_S0_M0,  12);
   addParamToTable(P_BKS1_58_BCVM_SDS_24_1          , MIL_OUT_BKS1_58_IUS_SA2,  E_P_MIL_H4_L19_S0_M0,  14);

   addParamToTable(P_BKS1_58_BCVM_SSCU_1_1          , MIL_OUT_BKS1_58_IUS_SA3,  E_P_MIL_H4_L19_S0_M0,  4);
   addParamToTable(P_BKS1_58_BCVM_SSCU_2_1          , MIL_OUT_BKS1_58_IUS_SA3,  E_P_MIL_H4_L19_S0_M0,  5);
   addParamToTable(P_BKS1_58_BCVM_SDS_25_1          , MIL_OUT_BKS1_58_IUS_SA3,  E_P_MIL_H4_L19_S0_M0,  9);
   addParamToTable(P_BKS1_58_BCVM_SCB_1             , MIL_OUT_BKS1_58_IUS_SA3,  E_P_MIL_H4_L19_S0_M0,  8);

   addParamToTable(P_BKS1_58_BCVM_SSCU_3_1          , MIL_OUT_BKS1_58_IUS_SA4,  E_P_MIL_H4_L19_S0_M0,  2);

   addParamToTable(P_BKS1_58_BCVM_MVI_SS_1       , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  1);
   addParamToTable(P_BKS1_58_BCVM_MPA_SS_1       , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  2);
   addParamToTable(P_BKS1_58_BCVM_MPA_SS1_1      , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  3);
   addParamToTable(P_BKS1_58_BCVM_MPA_SS2_1      , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  4);
   addParamToTable(P_BKS1_58_BCVM_MSK_SS_1       , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  5);
   addParamToTable(P_BKS1_58_BCVM_KS_1           , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L35_S0_M0,  6);
   addParamToTable(P_BKS1_58_BCVM_SDZU_1         , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  8);
   addParamToTable(P_BKS1_58_BCVM_TIMER_START_1  , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  9);
   addParamToTable(P_BKS1_58_BCVM_TIMER_STOP_1   , MIL_OUT_BKS1_58_IUS_SA5    ,  E_P_MIL_H4_L19_S0_M0,  10);


   addParamToTable(P_BKS2_58_BCVM_SDS_7_2       ,  MIL_OUT_BKS2_58_NVG_SA1   , E_P_MIL_H4_L19_S0_M0, 6   );
   addParamToTable(P_BKS2_58_BCVM_SDS_8_2       ,  MIL_OUT_BKS2_58_NVG_SA1   , E_P_MIL_H4_L19_S0_M0, 7   );
   addParamToTable(P_BKS2_58_BCVM_SDS_9_2       ,  MIL_OUT_BKS2_58_NVG_SA1   , E_P_MIL_H4_L19_S0_M0, 8   );
   addParamToTable(P_BKS2_58_BCVM_SDS_27_2      ,  MIL_OUT_BKS2_58_NVG_SA1   , E_P_MIL_H4_L19_S0_M0, 12  );
   addParamToTable(P_BKS2_58_BCVM_SCB_2        ,    MIL_OUT_BKS2_58_NVG_SA3  ,  E_P_MIL_H4_L19_S0_M0, 3 );
   addParamToTable(P_BKS2_58_BCVM_Operating_Time_2 ,  MIL_OUT_BKS2_58_NVG_SA1  ,  E_P_MIL_H4_L19_S0_M0, 10 );
   addParamToTable(P_BKS2_58_BCVM_SJS_BKS_2        ,  MIL_OUT_BKS2_58_NVG_SA1  ,  E_P_MIL_H4_L19_S0_M0, 11 );
   addParamToTable(P_BKS2_58_BCVM_MVI_SS_2         ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 1  );
   addParamToTable(P_BKS2_58_BCVM_MPA_SS_2         ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 2  );
   addParamToTable(P_BKS2_58_BCVM_MPA_SS1_2        ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 3  );
   addParamToTable(P_BKS2_58_BCVM_MPA_SS2_2        ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 4  );
   addParamToTable(P_BKS2_58_BCVM_MSK_SS_2         ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 5  );
   addParamToTable(P_BKS2_58_BCVM_KS_2             ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L35_S0_M0, 6  );
   addParamToTable(P_BKS2_58_BCVM_SDZU_2           ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 8  );
   addParamToTable(P_BKS2_58_BCVM_TIMESTART_2      ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 9  );
   addParamToTable(P_BKS2_58_BCVM_TIMESTOP_2       ,  MIL_OUT_BKS2_58_NVG_SA5  ,  E_P_MIL_H4_L19_S0_M0, 10 );


   addParamToTable(P_BKS3_58_BCVM_SDS_11_3       ,  MIL_OUT_BKS3_58_IUS_SA1   , E_P_MIL_H4_L19_S0_M0, 4   );
   addParamToTable(P_BKS3_58_BCVM_SDS_12_3       ,  MIL_OUT_BKS3_58_IUS_SA1   , E_P_MIL_H4_L19_S0_M0, 5   );
   addParamToTable(P_BKS3_58_BCVM_SDS_15_3       ,  MIL_OUT_BKS3_58_IUS_SA1   , E_P_MIL_H4_L19_S0_M0, 6   );
   addParamToTable(P_BKS3_58_BCVM_SDS_28_3       ,  MIL_OUT_BKS3_58_IUS_SA1   , E_P_MIL_H4_L19_S0_M0, 10  );

   addParamToTable(P_BKS3_58_BCVM_SDS_13_3       ,  MIL_OUT_BKS3_58_IUS_SA2   , E_P_MIL_H4_L19_S0_M0, 5   );
   addParamToTable(P_BKS3_58_BCVM_SDS_14_3       ,  MIL_OUT_BKS3_58_IUS_SA2   , E_P_MIL_H4_L19_S0_M0, 6   );
   addParamToTable(P_BKS3_58_BCVM_SCB_3        ,  MIL_OUT_BKS3_58_IUS_SA3  ,  E_P_MIL_H4_L19_S0_M0, 6 );

   addParamToTable(P_BKS3_58_BCVM_Operating_Time_3 ,  MIL_OUT_BKS3_58_IUS_SA1  ,  E_P_MIL_H4_L19_S0_M0, 7 );
   addParamToTable(P_BKS3_58_BCVM_SJS_BKS_3        ,  MIL_OUT_BKS3_58_IUS_SA1  ,  E_P_MIL_H4_L19_S0_M0, 8 );

   addParamToTable(P_BKS3_58_BCVM_MVI_SS_3        ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 1  );
   addParamToTable(P_BKS3_58_BCVM_MPA_SS_3        ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 2  );
   addParamToTable(P_BKS3_58_BCVM_MPA_SS1_3       ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 3  );
   addParamToTable(P_BKS3_58_BCVM_MPA_SS2_3       ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 4  );
   addParamToTable(P_BKS3_58_BCVM_MSK_SS_3        ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 5  );
   addParamToTable(P_BKS3_58_BCVM_KS_3            ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L35_S0_M0, 6  );
   addParamToTable(P_BKS3_58_BCVM_SDZU_3          ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 8  );
   addParamToTable(P_BKS3_58_BCVM_TIMESTART_3     ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 9  );
   addParamToTable(P_BKS3_58_BCVM_TIMESTOP_3      ,  MIL_OUT_BKS3_58_IUS_SA5  ,  E_P_MIL_H4_L19_S0_M0, 10 );



   addParamToTable(P_BKS4_58_BCVM_SDS_19_4          ,  MIL_OUT_BKS4_58_NVG_SA1   ,   E_P_MIL_H4_L19_S0_M0, 5 );
   addParamToTable(P_BKS4_58_BCVM_SDS_17_4          ,  MIL_OUT_BKS4_58_NVG_SA2   ,   E_P_MIL_H4_L19_S0_M0, 4 );
   addParamToTable(P_BKS4_58_BCVM_SDS_18_4          ,  MIL_OUT_BKS4_58_NVG_SA2   ,   E_P_MIL_H4_L19_S0_M0, 5 );
   addParamToTable(P_BKS4_58_BCVM_SDS_21_4          ,  MIL_OUT_BKS4_58_NVG_SA2   ,   E_P_MIL_H4_L19_S0_M0, 7 );

   addParamToTable(P_BKS4_58_BCVM_Operating_Time_4 ,  MIL_OUT_BKS4_58_NVG_SA1  ,  E_P_MIL_H4_L19_S0_M0, 7 );
   addParamToTable(P_BKS4_58_BCVM_SJS_BKS_4        ,  MIL_OUT_BKS4_58_NVG_SA1  ,  E_P_MIL_H4_L19_S0_M0, 8 );

   addParamToTable(P_BKS4_58_BCVM_SCB_4        ,  MIL_OUT_BKS4_58_NVG_SA3  ,  E_P_MIL_H4_L19_S0_M0, 4 );

   addParamToTable(P_BKS4_58_BCVM_MVI_SS_4        ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 1  );
   addParamToTable(P_BKS4_58_BCVM_MPA_SS_4        ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 2  );
   addParamToTable(P_BKS4_58_BCVM_MPA_SS1_4       ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 3  );
   addParamToTable(P_BKS4_58_BCVM_MPA_SS2_4       ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 4  );
   addParamToTable(P_BKS4_58_BCVM_MSK_SS_4        ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 5  );
   addParamToTable(P_BKS4_58_BCVM_KS_4            ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L35_S0_M0, 6  );
   addParamToTable(P_BKS4_58_BCVM_SDZU_4          ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 8  );
   addParamToTable(P_BKS4_58_BCVM_TIMESTART_4     ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 9  );
   addParamToTable(P_BKS4_58_BCVM_TIMESTOP_4      ,  MIL_OUT_BKS4_58_NVG_SA4  ,  E_P_MIL_H4_L19_S0_M0, 10 );

   addParamToTable(P_ARK_KUR         ,AR_OUT_ARK_58, E_P_AR_H29_L17_S1_M90,  0162)    ;
   addParamToTable(P_ARK_STATUS_KUR  ,AR_OUT_ARK_58, E_P_AR_H31_L30_S0_M0,   0162)    ;

   addParamToTable(P_BP_WORD1  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  1)    ;
   addParamToTable(P_BP_WORD2  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  2)    ;
   addParamToTable(P_BP_WORD3  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  3)    ;
   addParamToTable(P_BP_WORD4  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  4)    ;
   addParamToTable(P_BP_WORD5  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  5)    ;
   addParamToTable(P_BP_WORD6  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  6)    ;
   addParamToTable(P_BP_WORD7  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  7)    ;
   addParamToTable(P_BP_WORD8  ,MIL_IN_RSBN_BP_58_NVG_SA12, E_P_MIL_H4_L19_S0_M0,  8)    ;

   addParamToTable(P_BP_WORD1  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  1)    ;
   addParamToTable(P_BP_WORD2  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  2)    ;
   addParamToTable(P_BP_WORD3  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  3)    ;
   addParamToTable(P_BP_WORD4  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  4)    ;
   addParamToTable(P_BP_WORD5  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  5)    ;
   addParamToTable(P_BP_WORD6  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  6)    ;
   addParamToTable(P_BP_WORD7  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  7)    ;
   addParamToTable(P_BP_WORD8  ,MIL_OUT_RSBN_BP_58_NVG_SA16, E_P_MIL_H4_L19_S0_M0,  8)    ;

   addParamToTable(P_BP_WORD1  ,MIL_IN_RSBN_BNP_58_NVG_SA11, E_P_MIL_H4_L19_S0_M0,  1)    ;
   addParamToTable(P_BP_WORD2  ,MIL_IN_RSBN_BNP_58_NVG_SA11, E_P_MIL_H4_L19_S0_M0,  2)    ;
   addParamToTable(P_BP_WORD2  ,MIL_OUT_RSBN_BNP_58_NVG_SA15, E_P_MIL_H4_L19_S0_M0,  2)    ;
   addParamToTable(P_BP_WORD3  ,MIL_OUT_RSBN_BNP_58_NVG_SA15, E_P_MIL_H4_L19_S0_M0,  3)    ;
   addParamToTable(P_BP_WORD5  ,MIL_OUT_RSBN_BNP_58_NVG_SA6, E_P_MIL_H4_L19_S0_M0,  5)    ;
   addParamToTable(P_BP_WORD6  ,MIL_OUT_RSBN_BNP_58_NVG_SA6, E_P_MIL_H4_L19_S0_M0,  6)    ;
   
   //addParamToTable(P_BP_SS  ,   MIL_OUT_RSBN_BNP_58_NVG_SA6, E_P_MIL_H4_L19_S0_M0,  6)    ;
   //addParamToTable(P_BNP_SS ,   MIL_OUT_RSBN_BNP_58_NVG_SA6, E_P_MIL_H4_L19_S0_M0,  6)    ;
   
   addParamToTable(P_COUNT_KSU  ,AR_OUT_VSU_UCOKS, E_P_AR_H31_L9_S0_M0,  021)    ;

   addParamToTable(P_KSU_X_PED              ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,   05)    ;
   addParamToTable(P_KSU_X_PED_MAT          ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   05)    ;

   addParamToTable(P_KSU_XTPLINT          ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M16,  06)    ;
   addParamToTable(P_KSU_XTPLINT_MAT      ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,  06)    ;
   addParamToTable(P_KSU_XTPRINT          ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M16,  07)    ;
   addParamToTable(P_KSU_XTPRINT_MAT      ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   07)    ;

   addParamToTable(P_KSU_XTPLINT          ,AR_IN_KSU_UCOKS1, E_P_AR_H29_L14_S1_M16,  06)    ;
   addParamToTable(P_KSU_XTPLINT_MAT      ,AR_IN_KSU_UCOKS1, E_P_AR_H31_L30_S0_M0,  06)    ;
   addParamToTable(P_KSU_XTPRINT          ,AR_IN_KSU_UCOKS1, E_P_AR_H29_L14_S1_M16,  07)    ;
   addParamToTable(P_KSU_XTPRINT_MAT      ,AR_IN_KSU_UCOKS1, E_P_AR_H31_L30_S0_M0,  07)    ;

   addParamToTable(P_KSU_X_TAN              ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M64,  0171)    ;
   addParamToTable(P_KSU_X_TAN_MAT          ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,  0171)    ;
   addParamToTable(P_KSU_X_GAMMA            ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,  0172)    ;
   addParamToTable(P_KSU_X_GAMMA_MAT        ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,  0172)    ;

   addParamToTable(P_KSU_P_BRAKE_L_SS1     ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M190, 0173)    ;
   addParamToTable(P_KSU_P_BRAKE_L_SS1_MAT ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0173)    ;
   addParamToTable(P_KSU_P_BRAKE_R_SS1     ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M190, 0174)    ;
   addParamToTable(P_KSU_P_BRAKE_R_SS1_MAT ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0174)    ;
   addParamToTable(P_KSU_P_BRAKE_L_SS2     ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M190, 0175)    ;
   addParamToTable(P_KSU_P_BRAKE_L_SS2_MAT ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0175)    ;
   addParamToTable(P_KSU_P_BRAKE_R_SS2     ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M190, 0176)    ;
   addParamToTable(P_KSU_P_BRAKE_R_SS2_MAT ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0176)    ;
   addParamToTable(P_KSU_D_MRK             ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,  0214)    ;
   addParamToTable(P_KSU_D_MRK_MAT         ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0214)    ;

   addParamToTable(P_KSU_ALPHA_MAX_DOP     ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M90,  0240)    ;
   addParamToTable(P_KSU_ALPHA_MAX_DOP_MAT ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0240)    ;
   addParamToTable(P_KSU_ALPHA_MIN_DOP     ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M90,  061)    ;
   addParamToTable(P_KSU_ALPHA_MIN_DOP_MAT ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   061)    ;
   addParamToTable(P_KSU_NY_MAX_DOP        ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M8,   0113)    ;
   addParamToTable(P_KSU_NY_MAX_DOP_MAT    ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0113)    ;
   addParamToTable(P_KSU_NY_MIN_DOP        ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M4,   0300)    ;
   addParamToTable(P_KSU_NY_MIN_DOP_MAT    ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0300)    ;
   addParamToTable(P_KSU_BETA_MAX_DOP      ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M90,  0302)    ;
   addParamToTable(P_KSU_BETA_MAX_DOP_MAT  ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0302)    ;

   addParamToTable(P_KSU_ALFA_RUD_PRAV      ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,   055)    ;
   addParamToTable(P_KSU_ALFA_RUD_PRAV_MAT  ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   055)    ;
   addParamToTable(P_KSU_ALFA_RUD_LEV       ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,   054)    ;
   addParamToTable(P_KSU_ALFA_RUD_LEV_MAT   ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   054)    ;
   addParamToTable(P_KSU_ALFA_RUD_SR        ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,   0133)    ;
   addParamToTable(P_KSU_ALFA_RUD_SR_MAT    ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0133)    ;

   addParamToTable(P_KSU_UPR_PROD           ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M40,   051)    ;
   addParamToTable(P_KSU_UPR_PROD_MAT       ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   051)    ;
   addParamToTable(P_KSU_UPR_BOK            ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M20,   052)    ;
   addParamToTable(P_KSU_UPR_BOK_MAT        ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   052)    ;
   addParamToTable(P_KSU_DNY_SAU           ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M2,   057)    ;
   addParamToTable(P_KSU_DNY_SAU_MAT       ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   057)    ;

   addParamToTable(P_KSU_SBI_SRK3          ,AR_IN_KSU_SBI_58, E_P_AR_H29_L9_S0_M0,    0272)    ;
   addParamToTable(P_KSU_SBI_SRK3_MAT      ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0272)    ;
   addParamToTable(P_KSU_SBI_SRK5          ,AR_IN_KSU_SBI_58, E_P_AR_H29_L9_S0_M0,    0274)    ;
   addParamToTable(P_KSU_SBI_SRK5_MAT      ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0274)    ;

//   addParamToTable(P_KSU_X_TAN_NPU              ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M64,  0305)    ;
//   addParamToTable(P_KSU_X_TAN_MAT_NPU          ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0305)    ;
//   addParamToTable(P_KSU_X_GAMMA_NPU            ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,  0154)    ;
//   addParamToTable(P_KSU_X_GAMMA_MAT_NPU        ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0154)    ;
//   addParamToTable(P_KSU_ALFA_RUD_SR_NPU        ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,  0215)    ;
//   addParamToTable(P_KSU_ALFA_RUD_SR_MAT_NPU    ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0215)    ;
//   addParamToTable(P_KSU_X_PED_NPU              ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M32,  0155)    ;
//   addParamToTable(P_KSU_X_PED_MAT_NPU          ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0155)    ;
//   addParamToTable(P_KSU_XTPLINT_NPU            ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M16,  0301)    ;
//   addParamToTable(P_KSU_XTPLINT_MAT_NPU        ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0301)    ;
//   addParamToTable(P_KSU_XTPRINT_NPU            ,AR_IN_KSU_SBI_58, E_P_AR_H29_L14_S1_M16,  0302)    ;
//   addParamToTable(P_KSU_XTPRINT_MAT_NPU        ,AR_IN_KSU_SBI_58, E_P_AR_H31_L30_S0_M0,   0302)    ;
//   addParamToTable(P_VDU_SRK17                  ,AR_IN_KSU_SBI_58, E_P_AR_H29_L0_S0_M0,    0346)    ;

   addParamToTable(P_ISRP_SRK11  ,MIL_IN_RSBN_BNP_58_NVG_SA11, E_P_MIL_H4_L19_S0_M0,  1)    ;
   addParamToTable(P_ISRP_SRK21  ,MIL_IN_RSBN_BNP_58_NVG_SA11, E_P_MIL_H4_L19_S0_M0,  1)    ;
   addParamToTable(P_ISRP_SRK31  ,MIL_IN_RSBN_BNP_58_NVG_SA11, E_P_MIL_H4_L19_S0_M0,  1)    ;

   addParamToTable(P_BNP_SS               ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H4_L19_S0_M0,  1 )    ;
   addParamToTable(P_BNP_GOT_A            ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,5,   4 )    ;
   addParamToTable(P_BNP_DP_A             ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,6,   4 )    ;
   addParamToTable(P_BNP_GOT_D            ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,7,   4 )    ;
   addParamToTable(P_BNP_DP_D             ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,8,   4 )    ;
   addParamToTable(P_BNP_GOT_P            ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,9,   4 )    ;
   addParamToTable(P_BNP_DP_D             ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,10,  4 )    ;
   addParamToTable(P_BNP_GOT_D_TD         ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,11,  4 )    ;
   addParamToTable(P_BNP_DP_D_TD          ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,12,  4 )    ;
   addParamToTable(P_BNP_NO_CONTROL       ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,13,  4 )    ;
   addParamToTable(P_BNP_MODE_RSBN        ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,14,  4 )    ;
   addParamToTable(P_BNP_MODE_PRMG        ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,15,  4 )    ;
   addParamToTable(P_BNP_MODE_TACAN       ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,16,  4 )    ;
   addParamToTable(P_BNP_MODE_DME_P       ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,17,  4 )    ;
   addParamToTable(P_BNP_MODE_DME_N       ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,18,  4 )    ;
   addParamToTable(P_BNP_MODE_DME         ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,19,  4 )    ;
   addParamToTable(P_BNP_ARM              ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H4_L16_S1_M90,  5 )    ;
   addParamToTable(P_BNP_GOT_A            ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,19,  5 )    ;
   addParamToTable(P_BNP_DRM              ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H4_L18_S0_M250000,  6 )    ;
   addParamToTable(P_BNP_GOT_D            ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,19,  6 )    ;
   addParamToTable(P_BNP_EPS_K            ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H4_L16_S1_M42_53,  7 )    ;
   addParamToTable(P_BNP_GOT_EPS_K        ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,19,  7 )    ;
   addParamToTable(P_BNP_EPS_G            ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H4_L16_S1_M75_38,  8 )    ;
   addParamToTable(P_BNP_GOT_EPS_G        ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,19,  8 )    ;
   addParamToTable(P_BNP_EPS_R0           ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H4_L17_S0_M250000,  9 )    ;
   addParamToTable(P_BNP_GOT_R0           ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,19,  9 )    ;
   addParamToTable(P_BNP_DELTA_RPK        ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H4_L16_S1_M42_53,  10)    ;
   addParamToTable(P_BNP_RAZVOROT         ,MIL_OUT_RSBN_BNP_58_NVG_SA15,E_P_MIL_H17_L18_S0_M0,  10)    ;
   addParamToTable(P_BNP_GOT_DELTA_RPK    ,MIL_OUT_RSBN_BNP_58_NVG_SA15,toMIL,19,  10)    ;

   addParamToTable(CIMSS_A_NUM_BCVM_KSS        , -1, E_P_MIL_H4_L19_S0_M0 , 1 );
   addParamToTable(CIMSS_A_MASSIV_1_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 2 );
   addParamToTable(CIMSS_A_MASSIV_2_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 3 );
   addParamToTable(CIMSS_A_MASSIV_3_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 4 );
   addParamToTable(CIMSS_A_MASSIV_4_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 5 );
   addParamToTable(CIMSS_A_MASSIV_5_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 6 );
   addParamToTable(CIMSS_A_MASSIV_6_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 7 );
   addParamToTable(CIMSS_A_MASSIV_7_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 8 );
   addParamToTable(CIMSS_A_MASSIV_8_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 9 );
   addParamToTable(CIMSS_A_MASSIV_9_BCVM_KSS   , -1, E_P_MIL_H4_L19_S0_M0 , 10 );
   addParamToTable(CIMSS_A_MASSIV_10_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 11 );
   addParamToTable(CIMSS_A_MASSIV_11_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 12 );
   addParamToTable(CIMSS_A_MASSIV_12_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 13 );
   addParamToTable(CIMSS_A_MASSIV_13_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 14 );
   addParamToTable(CIMSS_A_MASSIV_14_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 15 );
   addParamToTable(CIMSS_A_MASSIV_15_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 16 );
   addParamToTable(CIMSS_A_MASSIV_16_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 17 );
   addParamToTable(CIMSS_A_MASSIV_17_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 18 );
   addParamToTable(CIMSS_A_MASSIV_18_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 19 );
   addParamToTable(CIMSS_A_MASSIV_19_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 20 );
   addParamToTable(CIMSS_A_MASSIV_20_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 21 );
   addParamToTable(CIMSS_A_MASSIV_21_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 22 );
   addParamToTable(CIMSS_A_MASSIV_22_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 23 );
   addParamToTable(CIMSS_A_MASSIV_23_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 24 );
   addParamToTable(CIMSS_A_MASSIV_24_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 25 );
   addParamToTable(CIMSS_A_MASSIV_25_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 26 );
   addParamToTable(CIMSS_A_MASSIV_26_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 27 );
   addParamToTable(CIMSS_A_MASSIV_27_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 28 );
   addParamToTable(CIMSS_A_MASSIV_28_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 29 );
   addParamToTable(CIMSS_A_MASSIV_29_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 30 );
   addParamToTable(CIMSS_A_MASSIV_30_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 31 );
   addParamToTable(CIMSS_A_MASSIV_31_BCVM_KSS  , -1, E_P_MIL_H4_L19_S0_M0 , 32 );


   addParamToTable(P_KSS_FREQ_ILS        ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L18_S0_M102_4, 01);
   addParamToTable(P_KSS_ZAD_NUM_SH_APDD ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L14_S0_M0    , 02);
   addParamToTable(P_KSS_NAV_REG         ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L25_S0_M0    , 03);
   addParamToTable(P_KSS_NAPR_POSSADKA   ,  AR_OUT_IKRL_1_1_BCVM, toAR, 24                , 03);
   addParamToTable(P_KSS_NAPR_3RD        ,  AR_OUT_IKRL_1_1_BCVM, toAR, 23                , 03);
   addParamToTable(P_KSS_METHOD_IN_MRSH  ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H22_L19_S0_M0    , 03);
   addParamToTable(P_KSS_NUM_M_MRSH      ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L26_S0_M0    , 04);
   addParamToTable(P_KSS_NUM_ISP_MRSH    ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H25_L19_S0_M0    , 04);
   addParamToTable(P_KSS_TYPE_KOR        ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H18_L16_S0_M0    , 04);
   addParamToTable(P_KSS_HPPM            ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L15_S0_M0    , 05);
   addParamToTable(P_KSS_NUMAEROVOZ      ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H14_L9_S0_M0     , 06);
   addParamToTable(P_KSS_LAM_ISP_AERO    ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L9_S1_M_PI_2 , 07);
   addParamToTable(P_KSS_FI_ISP_AERO     ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L9_S1_M_PI_2 , 010);
   addParamToTable(P_KSS_H_KR_ISP_AERO   ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L19_S0_M0    , 011);
   addParamToTable(P_KSS_KURS_ISP_AERO   ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L21_S0_M0    , 012);
   addParamToTable(P_KSS_D_ISP_AERO      ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H20_L12_S0_M25_6 , 013);
   addParamToTable(P_KSS_KURS_PR_PPM     ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L14_S0_M180  , 014);
   addParamToTable(P_KSS_D_PPM           ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L16_S0_M81_92, 015);
   addParamToTable(P_KSS_CUR_HOUR        ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L25_S0_M0    , 016); //033
   addParamToTable(P_KSS_CUR_MIN         ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H24_L19_S0_M0    , 016);
   addParamToTable(P_KSS_CUR_SEC         ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H18_L13_S0_M0    , 016);
   addParamToTable(P_KSS_FI              ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L14_S1_M_PI_2, 017);
   addParamToTable(P_KSS_LAM             ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L14_S1_M_PI_2, 020);
   addParamToTable(P_KSS_YEAR            ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L23_S0_M0    , 021);
   addParamToTable(P_KSS_MOUNTH          ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H22_L19_S0_M0    , 021);
   addParamToTable(P_KSS_DAY             ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H18_L14_S0_M0    , 021);
   addParamToTable(P_KSS_HOUR            ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L25_S0_M0    , 022);
   addParamToTable(P_KSS_MIN             ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H24_L19_S0_M0    , 022);
   addParamToTable(P_KSS_SEC             ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H18_L13_S0_M0    , 022);
   addParamToTable(P_KSS_REG_S70         ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L26_S0_M0    , 023);
   addParamToTable(P_KSS_OTKAZ_NASEM_SOST,  AR_OUT_IKRL_1_1_BCVM, toAR, 18                , 023);
   addParamToTable(P_KSS_REG_WORK_NC     ,  AR_OUT_IKRL_1_1_BCVM, toAR, 17                , 023);
   addParamToTable(P_KSS_OTKAZ_DMV1      ,  AR_OUT_IKRL_1_1_BCVM, toAR, 29                , 024);
   addParamToTable(P_KSS_OTKAZ_DMV2      ,  AR_OUT_IKRL_1_1_BCVM, toAR, 28                , 024);
   addParamToTable(P_KSS_OTKAZ_VIDEO1    ,  AR_OUT_IKRL_1_1_BCVM, toAR, 27                , 024);
   addParamToTable(P_KSS_OTKAZ_VIDEO1    ,  AR_OUT_IKRL_1_1_BCVM, toAR, 26                , 024);
   addParamToTable(P_KSS_OTKAZ_UPR1      ,  AR_OUT_IKRL_1_1_BCVM, toAR, 25                , 024);
   addParamToTable(P_KSS_OTKAZ_UPR2      ,  AR_OUT_IKRL_1_1_BCVM, toAR, 24                , 024);
   addParamToTable(P_KSS_OTKAZ_CIMSS_AN  ,  AR_OUT_IKRL_1_1_BCVM, toAR, 23                , 024);
   addParamToTable(P_KSS_CNT_ISPR        ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H28_L14_S0_M0     ,025);
   addParamToTable(P_KSS_CNT_OBMEN       ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H21_L14_S0_M0     ,026);
   addParamToTable(P_KSS_NUM_PP_UCOKS_ST ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L14_S0_M0     ,027);
   addParamToTable(P_KSS_NUM_PP_UCOKS_ML ,  AR_OUT_IKRL_1_1_BCVM, E_P_AR_H29_L14_S0_M0     ,030);

   addParamToTable(P_KSS_FREQ_ILS        ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L18_S0_M102_4, 01);
   addParamToTable(P_KSS_ZAD_NUM_SH_APDD ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L14_S0_M0    , 02);
   addParamToTable(P_KSS_NAV_REG         ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L25_S0_M0    , 03);
   addParamToTable(P_KSS_NAPR_POSSADKA   ,  AR_OUT_IKRL_2_1_BCVM, toAR, 24                , 03);
   addParamToTable(P_KSS_NAPR_3RD        ,  AR_OUT_IKRL_2_1_BCVM, toAR, 23                , 03);
   addParamToTable(P_KSS_METHOD_IN_MRSH  ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H22_L19_S0_M0    , 03);
   addParamToTable(P_KSS_NUM_M_MRSH      ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L26_S0_M0    , 04);
   addParamToTable(P_KSS_NUM_ISP_MRSH    ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H25_L19_S0_M0    , 04);
   addParamToTable(P_KSS_TYPE_KOR        ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H18_L16_S0_M0    , 04);
   addParamToTable(P_KSS_HPPM            ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L15_S0_M0    , 05);
   addParamToTable(P_KSS_NUMAEROVOZ      ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H14_L9_S0_M0     , 06);
   addParamToTable(P_KSS_LAM_ISP_AERO    ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L9_S1_M_PI_2 , 07);
   addParamToTable(P_KSS_FI_ISP_AERO     ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L9_S1_M_PI_2 , 010);
   addParamToTable(P_KSS_H_KR_ISP_AERO   ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L19_S0_M0    , 011);
   addParamToTable(P_KSS_KURS_ISP_AERO   ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L21_S0_M0    , 012);
   addParamToTable(P_KSS_D_ISP_AERO      ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H20_L12_S0_M25_6 , 013);
   addParamToTable(P_KSS_KURS_PR_PPM     ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L14_S0_M180  , 014);
   addParamToTable(P_KSS_D_PPM           ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L16_S0_M81_92, 015);
   addParamToTable(P_KSS_CUR_HOUR        ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L25_S0_M0    , 016); //033
   addParamToTable(P_KSS_CUR_MIN         ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H24_L19_S0_M0    , 016);
   addParamToTable(P_KSS_CUR_SEC         ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H18_L13_S0_M0    , 016);
   addParamToTable(P_KSS_FI              ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L14_S1_M_PI_2, 017);
   addParamToTable(P_KSS_LAM             ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L14_S1_M_PI_2, 020);
   addParamToTable(P_KSS_YEAR            ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L23_S0_M0    , 021);
   addParamToTable(P_KSS_MOUNTH          ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H22_L19_S0_M0    , 021);
   addParamToTable(P_KSS_DAY             ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H18_L14_S0_M0    , 021);
   addParamToTable(P_KSS_HOUR            ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L25_S0_M0    , 022);
   addParamToTable(P_KSS_MIN             ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H24_L19_S0_M0    , 022);
   addParamToTable(P_KSS_SEC             ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H18_L13_S0_M0    , 022);
   addParamToTable(P_KSS_REG_S70         ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L26_S0_M0    , 023);
   addParamToTable(P_KSS_OTKAZ_NASEM_SOST,  AR_OUT_IKRL_2_1_BCVM, toAR, 18                , 023);
   addParamToTable(P_KSS_REG_WORK_NC     ,  AR_OUT_IKRL_2_1_BCVM, toAR, 17                , 023);
   addParamToTable(P_KSS_OTKAZ_DMV1      ,  AR_OUT_IKRL_2_1_BCVM, toAR, 29                , 024);
   addParamToTable(P_KSS_OTKAZ_DMV2      ,  AR_OUT_IKRL_2_1_BCVM, toAR, 28                , 024);
   addParamToTable(P_KSS_OTKAZ_VIDEO1    ,  AR_OUT_IKRL_2_1_BCVM, toAR, 27                , 024);
   addParamToTable(P_KSS_OTKAZ_VIDEO1    ,  AR_OUT_IKRL_2_1_BCVM, toAR, 26                , 024);
   addParamToTable(P_KSS_OTKAZ_UPR1      ,  AR_OUT_IKRL_2_1_BCVM, toAR, 25                , 024);
   addParamToTable(P_KSS_OTKAZ_UPR2      ,  AR_OUT_IKRL_2_1_BCVM, toAR, 24                , 024);
   addParamToTable(P_KSS_OTKAZ_CIMSS_AN  ,  AR_OUT_IKRL_2_1_BCVM, toAR, 23                , 024);
   addParamToTable(P_KSS_CNT_ISPR        ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H28_L14_S0_M0     ,025);
   addParamToTable(P_KSS_CNT_OBMEN       ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H21_L14_S0_M0     ,026);
   addParamToTable(P_KSS_NUM_PP_UCOKS_ST ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L14_S0_M0     ,027);
   addParamToTable(P_KSS_NUM_PP_UCOKS_ML ,  AR_OUT_IKRL_2_1_BCVM, E_P_AR_H29_L14_S0_M0     ,030);


   addParamToTable(P_KSS_REG_S70         ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L7_S0_M0     , 1);
   addParamToTable(P_KSS_OTKAZ_NASEM_SOST,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 17                , 1);
   addParamToTable(P_KSS_OTKAZ_DMV1      ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 4                , 2);
   addParamToTable(P_KSS_OTKAZ_DMV2      ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 5                , 2);
   addParamToTable(P_KSS_OTKAZ_VIDEO1    ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 6                , 2);
   addParamToTable(P_KSS_OTKAZ_VIDEO1    ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 7                , 2);
   addParamToTable(P_KSS_OTKAZ_UPR1      ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 8                , 2);
   addParamToTable(P_KSS_OTKAZ_UPR2      ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 9                , 2);
   addParamToTable(P_KSS_FI              ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L19_S1_M_PI_2, 4);
   addParamToTable(P_KSS_LAM             ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L19_S1_M_PI_2, 5);
   addParamToTable(P_KSS_YEAR            ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L10_S0_M0    , 6);
   addParamToTable(P_KSS_MOUNTH          ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H11_L14_S0_M0   , 6);
   addParamToTable(P_KSS_DAY             ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H15_L19_S0_M0   , 6);
   addParamToTable(P_KSS_CUR_HOUR            ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L8_S0_M0     , 7);
   addParamToTable(P_KSS_CUR_MIN             ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H9_L14_S0_M0    , 7);
   addParamToTable(P_KSS_CUR_SEC             ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L9_S0_M0     , 8);
   addParamToTable(P_KSS_FREQ_ILS        ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L15_S0_M102_4, 10);
   addParamToTable(P_KSS_NAV_REG         ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L8_S0_M0     , 11);
   addParamToTable(P_KSS_ZAD_NUM_SH_APDD ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L19_S0_M0    , 12);
   addParamToTable(P_KSS_NUM_M_MRSH      ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L7_S0_M0     , 13);
   addParamToTable(P_KSS_NUM_ISP_MRSH    ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H8_L14_S0_M0    , 13);
   addParamToTable(P_KSS_TYPE_KOR        ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H15_L17_S0_M0   , 13);
   addParamToTable(P_KSS_NAPR_POSSADKA   ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 4                , 14);
   addParamToTable(P_KSS_HOUR            ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L8_S0_M0     , 15);
   addParamToTable(P_KSS_MIN             ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H9_L14_S0_M0    , 15);
   addParamToTable(P_KSS_SEC             ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L9_S0_M0     , 16);
   addParamToTable(P_KSS_ZAP_CS_IUS      ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 16               , 17);
   addParamToTable(P_KSS_METHOD_IN_MRSH  ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L7_S0_M0     , 18);
   addParamToTable(P_KSS_HPPM            ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L18_S0_M0    , 19);
   addParamToTable(P_KSS_NUMAEROVOZ      ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L9_S0_M0     , 20);
   addParamToTable(P_KSS_LAM_ISP_AERO    ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L24_S1_M_PI  , 21);
   addParamToTable(P_KSS_FI_ISP_AERO     ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L24_S1_M_PI  , 23);
   addParamToTable(P_KSS_H_KR_ISP_AERO   ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L14_S0_M0    , 25);
   addParamToTable(P_KSS_KURS_ISP_AERO   ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L12_S0_M0    , 26);
   addParamToTable(P_KSS_D_ISP_AERO      ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L12_S0_M25_6 , 27);
   addParamToTable(P_KSS_NAPR_3RD        ,  MIL_OUT_CIMSS_A_KSS_SA1, toMIL, 13                ,27);
   addParamToTable(P_KSS_KURS_PR_PPM     ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L19_S0_M180  , 28);
   addParamToTable(P_KSS_D_PPM           ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L17_S0_M81_92, 29);
   addParamToTable(P_KSS_VED_SR_POS      ,  MIL_OUT_CIMSS_A_KSS_SA1, E_P_MIL_H4_L6_S0_M0     , 30);

   //CO2010
   addParamToTable(P_CO2010_BCVM_OTKAZ_SO  , AR_OUT_CO2010, toAR,            11 , 0350);
   addParamToTable(P_CO2010_BCVM_OTKAZ_AFS , AR_OUT_CO2010, toAR,            12 , 0350) ;
   addParamToTable(P_CO2010_BCVM_OTKAZ_KLS , AR_OUT_CO2010, toAR,            14 , 0350);
   addParamToTable(P_CO2010_BCVM_MAT       , AR_OUT_CO2010, E_P_AR_H31_L30_S0_M0, 0350);

   addParamToTable(P_CO2010_KSU_USTP       , AR_IN_CO2010 , toAR,             11, 0250);
   addParamToTable(P_CO2010_KSU_H          , AR_IN_CO2010 , E_P_AR_H28_L14_S0_M0, 0250);
   addParamToTable(P_CO2010_KSU_SIGN_H     , AR_IN_CO2010 , toAR,             29, 0250);
   addParamToTable(P_CO2010_KSU_MAT        , AR_IN_CO2010 , E_P_AR_H31_L30_S0_M0, 0250);


    //! таблица описания упаковок
    //HAL::obj()->packTable.num = 104;

    addPackToTable(E_P_AR_H29_L12_S0_M4096,         TO_AR(29),  TO_AR(12),  0,  4096 * 2);
    addPackToTable(E_P_MIL_H4_L19_S0_M4096,         TO_MIL(4),  TO_MIL(19), 0,  4096 * 2);
    addPackToTable(E_P_AR_H29_L9_S1_M16384,         TO_AR(29),  TO_AR(9),  1,  16384 * 2);
    addPackToTable(E_P_AR_H29_L9_S0_M819_2,         TO_AR(29),  TO_AR(9),  0,  819.2 * 2);
    addPackToTable(E_P_AR_H29_L9_S0_M2365 ,         TO_AR(29),  TO_AR(9),  0,  2365 * 2);
    addPackToTable(E_P_AR_H29_L9_S0_M2_048,         TO_AR(29),  TO_AR(9),  0,  2.048 * 2);
    addPackToTable(E_P_AR_H29_L9_S1_M256,           TO_AR(29),  TO_AR(9),  1,  256 * 2);
    addPackToTable(E_P_AR_H29_L9_S1_M512,           TO_AR(29),  TO_AR(9),  1,  512 * 2);
    addPackToTable(E_P_AR_H29_L9_S1_M4096,          TO_AR(29),  TO_AR(9),  1,  4096 * 2);
    addPackToTable(E_P_AR_H29_L9_S1_M2_048,         TO_AR(29),  TO_AR(9),  1,  2.048 * 2);
    addPackToTable(E_P_AR_H29_L9_S0_M16384,         TO_AR(29),  TO_AR(9),  0,  16384 * 2);
    addPackToTable(E_P_AR_H29_L9_S1_M819_2,         TO_AR(29),  TO_AR(9),  1,  819.2 * 2);
    addPackToTable(E_P_AR_H29_L9_S0_M409_6,         TO_AR(29),  TO_AR(9),  0,  409.6 * 2);
    addPackToTable(E_P_AR_H29_L9_S0_M0,             TO_AR(29),  TO_AR(9),  0,  0);
    addPackToTable(E_P_AR_H29_L9_S0_M4096,          TO_AR(29),  TO_AR(9),  0,  4096 * 2);
    addPackToTable(E_P_AR_H31_L30_S0_M0,            TO_AR(31),  TO_AR(30), 0,  0);
    addPackToTable(E_P_MIL_H4_L19_S1_MMPI,          TO_MIL(4),  TO_MIL(19), 1,  M_PI  );
    addPackToTable(E_P_MIL_H4_L19_S1_M180,          TO_MIL(4),  TO_MIL(19), 1,  180.0  );
    addPackToTable(E_P_AR_H29_L11_S1_M11993_088,    TO_AR(29),  TO_AR(11),  1,  11993.088  );
    addPackToTable(E_P_AR_H29_L15_S1_M512,          TO_AR(29),  TO_AR(15),  1,  512  );
    addPackToTable(E_P_AR_H15_L9_S0_M0,             TO_AR(29),  TO_AR(9),   0,  0  );
    addPackToTable(E_P_AR_H28_L11_S0_M16384_2,      TO_AR(28),  TO_AR(11),  0,  16384 * 2  );
    addPackToTable(E_P_AR_H28_L11_S0_M7_86432,      TO_AR(28),  TO_AR(11),  0,  7.86432 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M16384,        TO_MIL(4),  TO_MIL(19), 1,  16384. * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M819_2,        TO_MIL(4),  TO_MIL(19), 1,  819.2 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M2048,         TO_MIL(4),  TO_MIL(19), 1,  2048 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M2_048,        TO_MIL(4),  TO_MIL(19), 1,  2.048 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M256,          TO_MIL(4),  TO_MIL(19), 1,  256. * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M512,          TO_MIL(4),  TO_MIL(19), 1,  512 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M90,           TO_MIL(4),  TO_MIL(19), 1,  90 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M8,            TO_MIL(4),  TO_MIL(19), 1,  8 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M4,            TO_MIL(4),  TO_MIL(19), 1,  4 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M812,          TO_MIL(4),  TO_MIL(19), 1,  812.2 * 2 );
    addPackToTable(E_P_MIL_H4_L19_S0_M0,            TO_MIL(4),  TO_MIL(19), 0,  0 );
    addPackToTable(E_P_MIL_H4_L19_S1_M32,           TO_MIL(4),  TO_MIL(19), 1,  32*2 );
    addPackToTable(E_P_AR_H29_L14_S0_M0,            TO_AR(29),  TO_AR(14),  0,  0 );
    addPackToTable(E_P_MIL_H4_L35_S1_M842_8658,     TO_D_MIL(4),TO_D_MIL(35),1,  842.8658*2 );
    addPackToTable(E_P_MIL_H4_L19_S1_M64,           TO_MIL(4),  TO_MIL(19), 1,  64 * 2 );
    addPackToTable(E_P_MIL_H4_L32_S1_M90,           TO_D_MIL(4),TO_D_MIL(32),1,  90 * 2 );
    addPackToTable(E_P_MIL_H4_L8_S0_M0,             TO_MIL(4),  TO_MIL(8),   1, 0 );
    addPackToTable(E_P_MIL_H4_L14_S0_M0,            TO_MIL(4),  TO_MIL(14),  0, 0 );
    addPackToTable(E_P_MIL_H4_L9_S0_M0,             TO_MIL(4),  TO_MIL(9),   0, 0 );
    addPackToTable(E_P_MIL_H4_L16_S0_M4096,         TO_MIL(4),  TO_MIL(16),  0, 4096 * 2 );
    addPackToTable(E_P_MIL_H4_L16_S0_M2_4576,       TO_MIL(4),  TO_MIL(16),  0, 2.4576 * 2 );
    addPackToTable(E_P_MIL_H4_L9_S0_M32,            TO_MIL(4),  TO_MIL(9),   0, 0);
    addPackToTable(E_P_MIL_H4_L9_S0_M0_48,          TO_MIL(4),  TO_MIL(9),    0,  0.48 * 2 );
    addPackToTable(E_P_MIL_H5_L12_S0_M128,          TO_MIL(5),  TO_MIL(12),   0, 128 * 2 );
    addPackToTable(E_P_MIL_H4_L13_S0_M0_512,        TO_MIL(4),  TO_MIL(13),   0, 0.512  * 2 );
    addPackToTable(E_P_MIL_H4_L14_S0_M1024,         TO_MIL(4),  TO_MIL(14),   0, 1024  * 2 );
    addPackToTable(E_P_AR_H18_L15_S0_M0,            TO_AR(18),    TO_AR(15),    0 , 0);
    addPackToTable(E_P_AR_H22_L19_S0_M0,            TO_AR(22),    TO_AR(19),    0 , 0);
    addPackToTable(E_P_AR_H26_L23_S0_M0,            TO_AR(26),    TO_AR(23),    0 , 0);
    addPackToTable(E_P_AR_H29_L27_S0_M0,            TO_AR(29),    TO_AR(27),    0 , 0);
    addPackToTable(E_P_AR_H28_L17_S1_M0_2,          TO_AR(28),    TO_AR(17),    1 , 0.2 * 2);
    addPackToTable(E_P_AR_H28_L17_S1_M0_4,          TO_AR(28),    TO_AR(17),    1 , 0.4 * 2);
    addPackToTable(E_P_MIL_H4_L18_S1_M64,           TO_MIL(4),    TO_MIL(18),   1 , 64 * 2);
    addPackToTable(E_P_MIL_H5_L18_S0_M128,          TO_MIL(5),    TO_MIL(18),   0 , 128 * 2);
    addPackToTable(E_P_MIL_H5_L18_S0_M256,          TO_MIL(5),    TO_MIL(18),   0 , 256 * 2);
    addPackToTable(E_P_MIL_H4_L18_S1_M32,           TO_MIL(4),    TO_MIL(18),   1 , 32 * 2);
    addPackToTable(E_P_MIL_H5_L18_S0_M16,           TO_MIL(5),    TO_MIL(18),   0 , 16 * 2);
    addPackToTable(E_P_MIL_H4_L18_S1_M128,          TO_MIL(4),    TO_MIL(18),   1 , 128 * 2);
    addPackToTable(E_P_MIL_H4_L18_S1_M0_125,        TO_MIL(4),    TO_MIL(18),   1 , 0.125 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M16384,        TO_AR(29),    TO_AR(14),    1 , 16384. * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M819_2,        TO_AR(29),    TO_AR(14),    1 , 819.2 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M2048,         TO_AR(29),    TO_AR(14),    1 , 2048 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M2_048,        TO_AR(29),    TO_AR(14),    1 , 2.048 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M256,          TO_AR(29),    TO_AR(14),    1 , 256 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M90,           TO_AR(29),    TO_AR(14),    1 , 90*2);
    addPackToTable(E_P_MIL_H4_L35_S1_MMPI,          TO_D_MIL(4),  TO_D_MIL(35), 1 , M_PI);
    addPackToTable(E_P_MIL_H4_L19_S0_M32760,        TO_MIL(4),    TO_MIL(19),   1 , 32760*2);
    addPackToTable(E_P_MIL_H4_L8_S1_M16384,         TO_MIL(4),    TO_MIL(8),    1 , 16384*2);
    addPackToTable(E_P_MIL_H4_L19_S1_M842_865,      TO_MIL(4),    TO_MIL(19),   1 , 842.865 * 2);
    addPackToTable(E_P_MIL_H9_L14_S0_M0,            TO_MIL(9),    TO_MIL(14),   0 , 0);
    addPackToTable(E_P_MIL_H4_L19_S0_M3276_8,       TO_MIL(9),    TO_MIL(14),   0 , 32768.8 * 2);
    addPackToTable(E_P_MIL_H4_L23_S0_M0_5,          TO_D_MIL(9),  TO_D_MIL(23), 0 , 0.5 * 2);
    addPackToTable(E_P_MIL_H4_L19_S0_M842_865,      TO_MIL(4),    TO_MIL(19),   0 , 842.865 * 2);
    addPackToTable(E_P_MIL_H5_L18_S0_M64,           TO_MIL(5),    TO_MIL(18),   0 , 64 * 2);
    addPackToTable(E_P_MIL_H4_L18_S1_M128,          TO_MIL(4),    TO_MIL(18),   1 , 128 * 2);
    addPackToTable(E_P_AR_H12_L11_S0_M0,            TO_AR(12),    TO_AR(11),    0 , 0);
    addPackToTable(E_P_AR_H14_L13_S0_M0,            TO_AR(14),    TO_AR(13),    0 , 0);
    addPackToTable(E_P_AR_H29_L15_S0_M0,            TO_AR(29),    TO_AR(15),    0 , 0);
    addPackToTable(E_P_AR_H29_L11_S1_M8,            TO_AR(29),    TO_AR(11),    0 , 0);
    addPackToTable(E_P_AR_H29_L9_S1_M65536,         TO_AR(29),    TO_AR(9),     1 , 65536 *2);
    addPackToTable(E_P_AR_H29_L14_S1_M512,          TO_AR(29),    TO_AR(14),    1 , 512 * 2);
    addPackToTable(E_P_AR_H29_L11_S1_M90,           TO_AR(29),    TO_AR(11),    1 , 90 * 2);
    addPackToTable(E_P_AR_H29_L11_S1_M1024,         TO_AR(29),    TO_AR(11),    1 , 1024 * 2);
    addPackToTable(E_P_AR_H29_L18_S1_M0_0000858,    TO_AR(29),    TO_AR(18),    1 , 0.0000858 * 2);
    addPackToTable(E_P_AR_H29_L12_S1_M8,            TO_AR(29),    TO_AR(12),    1 , 8 * 2);
    addPackToTable(E_P_AR_H29_L11_S1_M16384,        TO_AR(29),    TO_AR(11),    1 , 16384 * 2);
    addPackToTable(E_P_AR_H29_L11_S1_M2048,         TO_AR(29),    TO_AR(11),    1 , 2048 * 2);
    addPackToTable(E_P_AR_H29_L11_S0_M0,            TO_AR(29),    TO_AR(11),    0 , 0);
    addPackToTable(E_P_AR_H29_L9_S1_M90,            TO_AR(29),    TO_AR(9),     1 , 90 * 2);
    addPackToTable(E_P_AR_H28_L11_S0_M16384,        TO_AR(28),    TO_AR(11),    0 , 16384);
    addPackToTable(E_P_AR_H29_L17_S1_M0_2,          TO_AR(29),    TO_AR(17),    1 , 0.2 * 2);
    addPackToTable(E_P_AR_H29_L17_S1_M0_4,          TO_AR(29),    TO_AR(17),    1 , 0.4 * 2);
    addPackToTable(E_P_AR_H28_L11_S1_M7_86432,      TO_AR(28),    TO_AR(11),    1 , 7.86432 * 2);
    addPackToTable(E_P_AR_H29_L13_S1_M256,          TO_AR(29),    TO_AR(13),    1 , 256 * 2);
    addPackToTable(E_P_AR_H29_L0_S0_M0,             TO_AR(29),    TO_AR(1),     0 , 0);
    addPackToTable(E_P_AR_H28_L16_S0_M4096,         TO_AR(28),    TO_AR(16),    0 , 4096);
    addPackToTable(E_P_AR_H28_L16_S0_M163_84,       TO_AR(28),    TO_AR(16),    0 , 163.84);
    addPackToTable(E_P_AR_H28_L16_S0_M128,          TO_AR(28),    TO_AR(16),    0 , 128);
    addPackToTable(E_P_AR_H28_L16_S0_M2_4576,       TO_AR(28),    TO_AR(16),    0 , 2.4576);
    addPackToTable(E_P_AR_H28_L16_S0_M32,           TO_AR(28),    TO_AR(16),    0 , 32);
    addPackToTable(E_P_AR_H28_L16_S0_M1024,         TO_AR(28),    TO_AR(16),    0 , 1024);
    addPackToTable(E_P_AR_H28_L15_S0_M256,          TO_AR(28),    TO_AR(15),    0 , 256);
    addPackToTable(E_P_AR_H28_L15_S0_M0_125,        TO_AR(28),    TO_AR(15),    0 , 0.125);
    addPackToTable(E_P_MIL_H5_L18_S0_M4096,         TO_MIL(5),    TO_MIL(18),    0 , 4096);
    addPackToTable(E_P_MIL_H5_L18_S0_M4,            TO_MIL(5),    TO_MIL(18),    0 , 4);

    addPackToTable(E_P_MIL_H5_L18_S0_M32,           TO_MIL(5),    TO_MIL(18),    0 , 32);
    addPackToTable(E_P_MIL_H5_L18_S0_M1,            TO_MIL(5),    TO_MIL(18),    0 , 1);
    addPackToTable(E_P_MIL_H5_L18_S0_M512,         TO_MIL(5),    TO_MIL(18),    0 , 2*512);
    addPackToTable(E_P_MIL_H5_L19_S0_M512,         TO_MIL(5),    TO_MIL(19),    0 , 2*512);
    addPackToTable(E_P_MIL_H5_L18_S0_M1024,         TO_MIL(5),    TO_MIL(18),    0 , 2*1024);
    addPackToTable(E_P_MIL_H4_L35_S0_M0,            TO_D_MIL(4),  TO_D_MIL(35),    0 , 0);

    addPackToTable(E_P_AR_H28_L14_S0_M819_2 ,          TO_AR(28),    TO_AR(14),    0 , 819.2);
    addPackToTable(E_P_AR_H28_L14_S0_M2_048 ,          TO_AR(28),    TO_AR(14),    0 , 2.048);
    addPackToTable(E_P_AR_H29_L14_S1_M8     ,          TO_AR(29),    TO_AR(14),    1 , 8 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M4     ,          TO_AR(29),    TO_AR(14),    1 , 4 * 2);
    addPackToTable(E_P_AR_H28_L14_S0_M0     ,          TO_AR(28),    TO_AR(14),    0 , 0);
    addPackToTable(E_P_AR_H29_L14_S1_M2     ,          TO_AR(29),    TO_AR(14),    1 , 2 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M0_5   ,          TO_AR(29),    TO_AR(14),    1 , 0.5 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M1     ,          TO_AR(29),    TO_AR(14),    1 , 1 * 2);
    addPackToTable(E_P_AR_H28_L14_S0_M190   ,          TO_AR(28),    TO_AR(14),    0 , 190);
    addPackToTable(E_P_AR_H29_L15_S1_M64    ,          TO_AR(29),    TO_AR(15),    1 , 64 * 2);
    addPackToTable(E_P_AR_H29_L15_S1_M0_125 ,          TO_AR(29),    TO_AR(15),    1 , 0.125 * 2);
    addPackToTable( E_P_AR_H31_L1_S0_M0     ,          TO_AR(31),    TO_AR(1),    0 , 0);


    addPackToTable(E_P_MIL_H4_L18_S0_M0, TO_MIL(4),    TO_MIL(18),    0 , 0);
    //addPackToTable(E_P_MIL_H4_L35_S0_M0, TO_MIL(4),    TO_MIL(35),    0 , 0);
    addPackToTable(E_P_MIL_H4_L11_S0_M0, TO_MIL(4),    TO_MIL(11),    0 , 0);
    addPackToTable(E_P_MIL_H5_L19_S0_M190, TO_MIL(5),    TO_MIL(19),    0 , 190);

    addPackToTable(E_P_MIL_H4_L19_S1_M6,        TO_MIL(4),    TO_MIL(19),    1 , 6*2);
    addPackToTable(E_P_MIL_H4_L19_S1_M8,        TO_MIL(4),    TO_MIL(19),    1 , 8*2);
    addPackToTable(E_P_MIL_H4_L19_S1_M32,       TO_MIL(4),    TO_MIL(19),    1 , 32);
    addPackToTable(E_P_MIL_H5_L19_S0_M32,       TO_MIL(5),    TO_MIL(19),    0 , 32);
    addPackToTable(E_P_MIL_H5_L19_S0_M16,       TO_MIL(5),    TO_MIL(19),    0 , 16*2);
    addPackToTable(E_P_AR_H28_L1_S0_M0,         TO_AR(28),      TO_AR(1),0,0);
    addPackToTable(E_P_AR_H28_L12_S0_M0,        TO_AR(28),      TO_AR(12),0,0);
    addPackToTable(E_P_MIL_H4_L35_S1_M180,      TO_D_MIL(4),    TO_D_MIL(35),1,180);



    addPackToTable(E_P_AR_H29_L14_S1_M81_92    , TO_AR(29),TO_AR(14), 1, 81.92 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M1024    , TO_AR(29),TO_AR(14), 1, 1024 * 2);
    addPackToTable(E_P_AR_H29_L14_S1_M4096     , TO_AR(29),TO_AR(14), 1, 2*4096);
    addPackToTable(E_P_AR_H29_L14_S0_M2048     , TO_AR(29),TO_AR(14), 0, 2048);
    addPackToTable(E_P_AR_H29_L14_S1_M64       , TO_AR(29),TO_AR(14), 1, 2*64);
    addPackToTable(E_P_AR_H29_L14_S0_M2        , TO_AR(29),TO_AR(14), 0, 2);
    addPackToTable(E_P_MIL_H7_L14_S0_M0        , TO_MIL(7),TO_MIL(14), 0, 0);
    addPackToTable(E_P_MIL_H5_L19_S0_M128      , TO_MIL(5),TO_MIL(19), 0, 128);

    addPackToTable(E_P_AR_H28_L9_S0_M64        ,TO_AR(28),TO_AR(9),  0,  64*2);
    addPackToTable(E_P_AR_H28_L9_S0_M512       ,TO_AR(28),TO_AR(9),  0,  512*2);
    addPackToTable(E_P_AR_H24_L9_S0_M0         ,TO_AR(24),TO_AR(9),  0,  0);
    addPackToTable(E_P_AR_H28_L19_S0_M64       ,TO_AR(28),TO_AR(19), 0, 64 * 2);
    addPackToTable(E_P_AR_H29_L19_S1_M256      ,TO_AR(29),TO_AR(19), 1, 256 * 2);
    addPackToTable(E_P_AR_H28_L19_S0_M128      ,TO_AR(28),TO_AR(19), 0, 128*2);
    addPackToTable(E_P_MIL_H5_L19_S0_M512      ,TO_MIL(5),TO_AR(19), 0, 512);
    addPackToTable(E_P_MIL_H5_L11_S0_M64       ,TO_MIL(5),TO_AR(11), 0, 64);
    addPackToTable(E_P_MIL_H4_L16_S0_M32      ,TO_MIL(4),TO_MIL(16), 0, 32);
    addPackToTable(E_P_MIL_H4_L16_S0_M32       ,TO_MIL(4),TO_MIL(16),0, 32);
    addPackToTable(E_P_AR_H31_L9_S0_M0         ,TO_AR(31),TO_AR(9),  0, 0);
    addPackToTable(E_P_AR_H28_L11_S0_M0        ,TO_AR(28),TO_AR(11),  0, 0);


    addPackToTable(E_P_MIL_H4_L19_S1_M0_5      ,TO_MIL(4),TO_MIL(19), 1, 2*0.5  );
    addPackToTable(E_P_MIL_H4_L19_S1_M1024     ,TO_MIL(4),TO_MIL(19), 1, 2*1024 );
    addPackToTable(E_P_MIL_H4_L19_S1_M2        ,TO_MIL(4),TO_MIL(19), 1, 2*2    );
    addPackToTable(E_P_MIL_H4_L19_S1_M40       ,TO_MIL(4),TO_MIL(19), 1, 2*40   );
    addPackToTable(E_P_MIL_H4_L19_S1_M20       ,TO_MIL(4),TO_MIL(19), 1, 2*20   );
    addPackToTable(E_P_MIL_H4_L19_S1_M190      ,TO_MIL(4),TO_MIL(19), 1, 2*190  );
    addPackToTable(E_P_MIL_H4_L19_S1_M16       ,TO_MIL(4),TO_MIL(19), 1, 2*16   );
    addPackToTable(E_P_MIL_H4_L19_S1_M5        ,TO_MIL(4),TO_MIL(19), 1, 2*5    );
    addPackToTable(E_P_MIL_H4_L19_S1_M8192     ,TO_MIL(4),TO_MIL(19), 1, 2*8192 );
    addPackToTable(E_P_MIL_H4_L19_S1_M160     , TO_MIL(4),TO_MIL(19), 1, 2*160 );
    addPackToTable(E_P_MIL_H4_L15_S1_M64     ,  TO_MIL(4),TO_MIL(15), 1, 2*64 );
    addPackToTable(E_P_AR_H31_L9_S0_M0     ,    TO_AR(29),TO_AR(9), 1, 2*64 );

    addPackToTable(E_P_AR_H29_L14_S1_M40     ,    TO_AR(29),TO_AR(14), 1, 2*40 );
    addPackToTable(E_P_AR_H29_L14_S1_M20     ,    TO_AR(29),TO_AR(14), 1, 2*20 );
    addPackToTable(E_P_AR_H29_L14_S1_M32     ,    TO_AR(29),TO_AR(14), 1, 2*32);

    addPackToTable(E_P_AR_H31_L14_S0_M0        ,TO_AR(31),TO_AR(14),  0, 0);
    addPackToTable(E_P_AR_H29_L14_S1_M0        ,TO_AR(29),TO_AR(14),  1, 0);
    addPackToTable(E_P_AR_H29_L14_S1_M431_16   ,TO_AR(29),TO_AR(14),  1, 2*431.16);
    addPackToTable(E_P_AR_H29_L14_S1_M40_02    ,TO_AR(29),TO_AR(14),  1, 2*40.02);
    addPackToTable(E_P_AR_H29_L14_S1_M1233_73  ,TO_AR(29),TO_AR(14),  1, 2*1233.73);
    addPackToTable(E_P_AR_H29_L14_S1_M259_36   ,TO_AR(29),TO_AR(14),  1, 2*259.36);
    addPackToTable(E_P_AR_H29_L14_S1_431_16    ,TO_AR(29),TO_AR(14),  1, 2*431.16);

    addPackToTable(E_P_AR_H30_L9_S1_M16384    ,TO_AR(30),TO_AR(9),  1, 2*16384);
    addPackToTable(E_P_AR_H30_L9_S1_M819_2    ,TO_AR(30),TO_AR(9),  1, 2*819.2);
    addPackToTable(E_P_AR_H30_L9_S1_M2365     ,TO_AR(30),TO_AR(9),  1, 2*2365);
    addPackToTable(E_P_AR_H30_L9_S1_M2_048    ,TO_AR(30),TO_AR(9),  1, 2*2.048);
    addPackToTable(E_P_AR_H30_L9_S1_M256      ,TO_AR(30),TO_AR(9),  1, 2*256);
    addPackToTable(E_P_AR_H30_L9_S1_M4096     ,TO_AR(30),TO_AR(9),  1, 2*4096);
    addPackToTable(E_P_AR_H30_L9_S1_M409_6    ,TO_AR(30),TO_AR(9),  1, 2*409.6);
    addPackToTable(E_P_AR_H30_L9_S1_M512      ,TO_AR(30),TO_AR(9),  1, 2*512);
    addPackToTable(E_P_AR_H29_L14_S1_M6      , TO_AR(29),TO_AR(14), 1, 2*6);


    addPackToTable(E_P_MIL_H4_L16_S1_M90      , TO_MIL(4),TO_MIL(16), 1 , 2 * 90       );
    addPackToTable(E_P_MIL_H4_L18_S0_M250000  , TO_MIL(4),TO_MIL(18), 0 , 2 * 250000   );
    addPackToTable(E_P_MIL_H4_L16_S1_M42_53   , TO_MIL(4),TO_MIL(16), 1 , 2 * 42.53    );
    addPackToTable(E_P_MIL_H4_L16_S1_M75_38   , TO_MIL(4),TO_MIL(16), 1 , 2 * 75.38    );
    addPackToTable(E_P_MIL_H4_L17_S0_M250000  , TO_MIL(4),TO_MIL(17), 0 , 2 * 250000   );
    addPackToTable(E_P_MIL_H4_L16_S1_M42_53   , TO_MIL(4),TO_MIL(16), 1 , 2 * 42.53    );
    addPackToTable(E_P_MIL_H17_L18_S0_M0      , TO_MIL(17),TO_MIL(18), 0 , 0            );
    addPackToTable(E_P_AR_H29_L14_S1_M16      , TO_AR(29),TO_AR(14), 1 , 2 * 16            );


    addPackToTable(E_P_AR_H29_L18_S0_M102_4   , TO_AR(29),TO_AR(18), 0, 2 * 102.4);
    addPackToTable(E_P_AR_H29_L25_S0_M0       , TO_AR(29),TO_AR(25), 0, 0 );
    addPackToTable(E_P_AR_H29_L26_S0_M0       , TO_AR(29),TO_AR(26), 0, 0 );
    addPackToTable(E_P_AR_H25_L19_S0_M0       , TO_AR(25),TO_AR(19), 0, 0 );
    addPackToTable(E_P_AR_H18_L16_S0_M0       , TO_AR(18),TO_AR(16), 0, 0 );
    addPackToTable(E_P_AR_H14_L9_S0_M0        , TO_AR(14),TO_AR(9) , 0, 0 );
    addPackToTable(E_P_AR_H29_L9_S1_M_PI_2    , TO_AR(29),TO_AR(9) , 1, 2 * M_PI/2.);
    addPackToTable(E_P_AR_H29_L19_S0_M0       , TO_AR(29),TO_AR(19), 0, 0 );
    addPackToTable(E_P_AR_H29_L21_S0_M0       , TO_AR(29),TO_AR(21), 0, 0 );
    addPackToTable(E_P_AR_H20_L12_S0_M25_6    , TO_AR(20),TO_AR(12), 0, 2 * 25.6);
    addPackToTable(E_P_AR_H29_L14_S0_M180     , TO_AR(29),TO_AR(14), 0, 2 * 180);
    addPackToTable(E_P_AR_H29_L16_S0_M81_92   , TO_AR(29),TO_AR(16), 0, 2 * 81.92);
    addPackToTable(E_P_AR_H24_L19_S0_M0       , TO_AR(24),TO_AR(19), 0, 0 );
    addPackToTable(E_P_AR_H18_L13_S0_M0       , TO_AR(18),TO_AR(13), 0, 0 );
    addPackToTable(E_P_AR_H29_L14_S1_M_PI_2   , TO_AR(29),TO_AR(14), 1, 2 * M_PI/2.);
    addPackToTable(E_P_AR_H29_L23_S0_M0       , TO_AR(29),TO_AR(23), 0, 0 );
    addPackToTable(E_P_AR_H18_L14_S0_M0       , TO_AR(18),TO_AR(14), 0, 0 );
    addPackToTable(E_P_AR_H29_L25_S0_M0       , TO_AR(29),TO_AR(25), 0, 0 );
    addPackToTable(E_P_AR_H24_L19_S0_M0       , TO_AR(24),TO_AR(19), 0, 0 );
    addPackToTable(E_P_AR_H18_L13_S0_M0       , TO_AR(18),TO_AR(13), 0, 0 );
    addPackToTable(E_P_AR_H29_L26_S0_M0       , TO_AR(29),TO_AR(26), 0, 0 );
    addPackToTable(E_P_AR_H21_L14_S0_M0       , TO_AR(21),TO_AR(14), 0, 0 );
    addPackToTable(E_P_AR_H32_L1_S0_M0        , TO_AR(32),TO_AR(1) , 0, 0 );
    addPackToTable(E_P_AR_H29_L1_S0_M0        , TO_AR(29),TO_AR(1) , 0, 0 );
    addPackToTable(E_P_AR_H29_L14_S1_M190     , TO_AR(29),TO_AR(1) , 1, 2 * 190);


}

void HAL::addParamToTable(uint32_t idParam,int16_t idCh,uint16_t idPackCh,uint16_t idParamCh, bool bit  )
{
    HAL* t = HAL::obj();

    uint16_t index = t->paramTable.param[idParam].num;
    t->paramTable.param[idParam].idCh[index]        = idCh;
    t->paramTable.param[idParam].idPackCh[index]    = idPackCh;
    t->paramTable.param[idParam].idParamCh[index]   = idParamCh;
    if(bit == true)
        t->paramTable.param[idParam].bit[index]     = 1;

    t->paramTable.param[idParam].num++;

    if((HAL::obj()->paramTable.num-1)<idParam)
        HAL::obj()->paramTable.num = idParam+1;
}
void HAL::addParamToTable(uint32_t idParam,int16_t idCh,uint8_t (*idPackCh)(uint8_t),uint8_t value, uint16_t idParamCh )
{
    HAL* t = HAL::obj();

    uint16_t index = t->paramTable.param[idParam].num;
    t->paramTable.param[idParam].idCh[index]        = idCh;
    t->paramTable.param[idParam].idPackCh[index]    = idPackCh(value);
    t->paramTable.param[idParam].idParamCh[index]   = idParamCh;
    t->paramTable.param[idParam].bit[index]     = 1;

    t->paramTable.param[idParam].num++;

    if((HAL::obj()->paramTable.num-1)<idParam)
        HAL::obj()->paramTable.num = idParam+1;
}
void HAL::addPackToTable(uint32_t idParam, uint8_t hBit, uint8_t lBit, uint8_t sign, float scale)
{
    packTable.packer[idParam].hBit  = hBit;
    packTable.packer[idParam].lBit  = lBit;
    packTable.packer[idParam].sign  = sign;
    packTable.packer[idParam].scale = scale;

    HAL::obj()->packTable.num++;
}
//! загрузка прошивок
void HAL::downloadFirmware()
{
    std::cout << "========================================" << std::endl;
    std::cout<<std::setw(30)<<std::left<<"HAL: download firmware .... "<<std::endl;
}
/*void HAL::updateVerOS()
{
    std::cout << "========================================" << std::endl;
        std::cout<<std::setw(30)<<std::left<<"HAL: checking to update image OS .... "<<std::endl;

        int statusVer = ERROR, statusIm = ERROR, statusBoot = ERROR;
        char strVer[8] = "";

        std::string fileOsImage = "vxWorksPV";
        std::string fileSymbol  = "vxWorksPV.sym";
        std::string fileBootrom = "bootromPV.sys";
        if(HAL::obj()->typeCurrentNode == E_NODE_CV)
        {
            fileOsImage = "vxWorksCV";
            fileSymbol  = "vxWorksCV.sym";
            fileBootrom = "bootromCV.sys";
        }

        //! считываем номер версии из файла
        bool flag = HAL::obj()->readConfFromFile("/ata0a/ver.txt", (uintptr_t*)strVer, 8);
        if(flag!=true)
            std::cout<<std::setw(30)<<std::left<<"HAL: can`t read prevoius version"<<std::endl;

        std::string timeStr = "/ata0a/v_" + std::string(strVer);

        //! копируем новую версию в vxWorks
        int fd = open ("/ata0a/ver.txt", 0x201, 0644);
        if(fd != ERROR)
        {
            statusVer = tftpCopy ("host", 0, "ver.txt", "get", "ascii", fd);
            if(statusVer == ERROR)
                std::cout<<std::setw(30)<<std::left<<"HAL: can`t download version tag"<<std::endl;
            close(fd);
        }
        //! копируем новую версию в vxWorks
        fd = open ("/ata0a/vxWorksUp", 0x201, 0644);
        if(fd != ERROR)
        {
            statusIm = tftpCopy ("host", 0, (char*)fileOsImage.c_str(), "get", "binary", fd);
            if(statusIm == ERROR && statusVer == ERROR)
                std::cout<<std::setw(30)<<std::left<<"HAL: can`t download image"<<std::endl;
            close(fd);
        }
        //! копируем новую версию в vxWorks
        fd = open ("/ata0a/bootromUp.sys", 0x201, 0644);
        if(fd != ERROR)
        {
            statusBoot = tftpCopy ("host", 0, (char*)fileBootrom.c_str(), "get", "binary", fd);
            if(statusBoot == ERROR && statusVer == ERROR)
               std::cout<<std::setw(30)<<std::left<<"HAL: can`t download bootrom.sys"<<std::endl;
            close(fd);
        }
        //! копируем новую версию в vxWorks
        fd = open ("/ata0a/vxWorks.sym", 0x201, 0644);
        if(fd != ERROR)
        {
            statusIm = tftpCopy ("host", 0, (char*)fileSymbol.c_str(), "get", "binary", fd);
            if(statusIm == ERROR && statusVer == ERROR)
               std::cout<<std::setw(30)<<std::left<<"HAL: can`t download symbols"<<std::endl;
            close(fd);
        }


        if(statusIm == OK && statusVer == OK)
        {
            std::cout<<std::setw(30)<<std::left<<"HAL: burning new image OS .... "<<std::endl;
            //! переименовываем старую версию
            fd = open( "/ata0a/vxWorks", 0x201, 0644 );
            int status = mv("/ata0a/vxWorks", timeStr.c_str());
            if(fd!=ERROR)
                close(fd);
            fd = open( "/ata0a/vxWorksUp", 0x201, 0644 );
            status = mv("/ata0a/vxWorksUp", "/ata0a/vxWorks");
            if(fd!=ERROR)
                close(fd);

            timeStr = "/ata0a/bootrom.sys" + std::string(strVer);
            fd = open( "/ata0a/bootrom.sys", 0x201, 0644 );
            int statusB = mv("/ata0a/bootrom.sys", timeStr.c_str());
            if(fd!=ERROR)
               close(fd);
            fd = open( "/ata0a/bootromUp.sys", 0x201, 0644 );
            statusB = mv("/ata0a/bootromUp.sys", "/ata0a/bootrom.sys");
            if(fd!=ERROR)
                close(fd);

            if(status == OK)
                std::cout<<std::setw(30)<<std::left<<"HAL: updating  - OK"<<std::endl;
            else
                std::cout<<std::setw(30)<<std::left<<"HAL: updating  - FAIL"<<std::endl;
            std::cout << "========================================" << std::endl;
            reboot(0);
        }else if(statusIm == ERROR && statusVer == OK)
        {
            char prevVer[8]="";
            std::cout<<std::setw(30)<<std::left<<"HAL: restoring prev image OS .... "<<std::endl;
            //! переименовываем старую версию
            fd = open( "/ata0a/vxWorks", 0x201, 0644 );
            int status = mv("/ata0a/vxWorks", timeStr.c_str());
            if(fd!=ERROR)
                close(fd);
            //! считываем номер версии из файла
            bool flag = HAL::obj()->readConfFromFile("/ata0a/ver.txt", (uintptr_t*)prevVer, 8);
            if(flag!=true)
                 std::cout<<std::setw(30)<<std::left<<"HAL: can`t read file with version"<<std::endl;

            std::string prevStr = "/ata0a/v_" + std::string(prevVer);
            //! пытаюсь найти старый файл
            fd = open( prevStr.c_str(), 0x201, 0644 );
            status = mv(prevStr.c_str(), "/ata0a/vxWorks");
            if(fd!=ERROR)
                close(fd);
            if(status == OK)
                std::cout<<std::setw(30)<<std::left<<"HAL: restore   - OK"<<std::endl;
            else
                std::cout<<std::setw(30)<<std::left<<"HAL: restore   - FAIL"<<std::endl;
            std::cout << "========================================" << std::endl;

            reboot(0);
        }else
        {
            std::cout<<std::setw(30)<<std::left<<"HAL: nothing to update - OK"<<std::endl;
            std::cout << "========================================" << std::endl;
        }
}*/
void HAL::updateVerConf()
{   
    uploadFile("/ata0a/conf/","ethTable.bin");
    uploadFile("/ata0a/conf/","ethPortTable.bin");
    uploadFile("/ata0a/conf/","nameTable.bin");
    uploadFile("/ata0a/conf/","confISA.bin");
    uploadFile("/ata0a/conf/","packTable.bin");
    uploadFile("/ata0a/conf/","paramTable.bin");
    uploadFile("/ata0a/conf/","chTable.bin");   
}
void HAL::updateVerOS()
{
    std::cout << "========================================" << std::endl;
    std::cout<<std::setw(30)<<std::left<<"HAL: checking to update image OS .... "<<std::endl;

    int statusVer = ERROR, statusIm = ERROR, statusBoot = ERROR;
    char strVer[8] = "";

    std::string fileOsImage = "vxWorksPV";
    std::string fileSymbol  = "vxWorksPV.sym";
    std::string fileBootrom = "bootromPV.sys";
    
    
    if(HAL::obj()->typeCurrentNode == E_NODE_CV)
    {
        fileOsImage = "vxWorksCV";
        fileSymbol  = "vxWorksCV.sym";
        fileBootrom = "bootromCV.sys";
    }

    //! считываем номер версии из файла
    bool flag = HAL::obj()->readConfFromFile("/ata0a/ver.txt", (uintptr_t*)strVer, 8);
    if(flag!=true)
        std::cout<<std::setw(30)<<std::left<<"HAL: can`t read prevoius version"<<std::endl;

    std::string timeStr = "/ata0a/v_" + std::string(strVer);

    //! копируем новую версию в vxWorks
    int fd = open ("/ata0a/ver.txt", 0x201, 0644);
    if(fd != ERROR)
    {
        statusVer = tftpCopy ("host", 0, "ver.txt", "get", "ascii", fd);
        if(statusVer == ERROR)
            std::cout<<std::setw(30)<<std::left<<"HAL: can`t download version tag"<<std::endl;
        close(fd);
    }
   
    //! копируем новую версию в vxWorks
    fd = open ("/ata0a/vxWorksUp", 0x201, 0644);
    if(fd != ERROR)
    {
        statusIm = tftpCopy ("host", 0, (char*)fileOsImage.c_str(), "get", "binary", fd);
        if(statusIm == ERROR && statusVer == ERROR)
            std::cout<<std::setw(30)<<std::left<<"HAL: can`t download image"<<std::endl;
        close(fd);
    }
    //! копируем новую версию в vxWorks
    fd = open ("/ata0a/bootromUp.sys", 0x201, 0644);
    if(fd != ERROR)
    {
        statusBoot = tftpCopy ("host", 0, (char*)fileBootrom.c_str(), "get", "binary", fd);
        if(statusBoot == ERROR && statusVer == ERROR)
           std::cout<<std::setw(30)<<std::left<<"HAL: can`t download bootrom.sys"<<std::endl;
        close(fd);
    }
    //! копируем новую версию в vxWorks
    fd = open ("/ata0a/vxWorks.sym", 0x201, 0644);
    if(fd != ERROR)
    {
        statusIm = tftpCopy ("host", 0, (char*)fileSymbol.c_str(), "get", "binary", fd);
        if(statusIm == ERROR && statusVer == ERROR)
           std::cout<<std::setw(30)<<std::left<<"HAL: can`t download symbols"<<std::endl;
        close(fd);
    }
    if(statusIm == OK && statusVer == OK)
    {
        std::cout<<std::setw(30)<<std::left<<"HAL: burning new image OS .... "<<std::endl;
        //! переименовываем старую версию
        fd = open( "/ata0a/vxWorks", 0x201, 0644 );
        int status = mv("/ata0a/vxWorks", timeStr.c_str());
        if(fd!=ERROR)
            close(fd);
        fd = open( "/ata0a/vxWorksUp", 0x201, 0644 );
        status = mv("/ata0a/vxWorksUp", "/ata0a/vxWorks");
        if(fd!=ERROR)
            close(fd);

        timeStr = "/ata0a/bootrom.sys" + std::string(strVer);
        fd = open( "/ata0a/bootrom.sys", 0x201, 0644 );
        int statusB = mv("/ata0a/bootrom.sys", timeStr.c_str());
        if(fd!=ERROR)
           close(fd);
        fd = open( "/ata0a/bootromUp.sys", 0x201, 0644 );
        statusB = mv("/ata0a/bootromUp.sys", "/ata0a/bootrom.sys");
        if(fd!=ERROR)
            close(fd);

        if(status == OK)
            std::cout<<std::setw(30)<<std::left<<"HAL: updating  - OK"<<std::endl;
        else
            std::cout<<std::setw(30)<<std::left<<"HAL: updating  - FAIL"<<std::endl;
        std::cout << "========================================" << std::endl;
        reboot(0);
    }else if(statusIm == ERROR && statusVer == OK)
    {
        char prevVer[8]="";
        std::cout<<std::setw(30)<<std::left<<"HAL: restoring prev image OS .... "<<std::endl;
        //! переименовываем старую версию
        fd = open( "/ata0a/vxWorks", 0x201, 0644 );
        int status = mv("/ata0a/vxWorks", timeStr.c_str());
        if(fd!=ERROR)
            close(fd);
        //! считываем номер версии из файла
        bool flag = HAL::obj()->readConfFromFile("/ata0a/ver.txt", (uintptr_t*)prevVer, 8);
        if(flag!=true)
             std::cout<<std::setw(30)<<std::left<<"HAL: can`t read file with version"<<std::endl;

        std::string prevStr = "/ata0a/v_" + std::string(prevVer);
        //! пытаюсь найти старый файл
        fd = open( prevStr.c_str(), 0x201, 0644 );
        status = mv(prevStr.c_str(), "/ata0a/vxWorks");
        if(fd!=ERROR)
            close(fd);
        if(status == OK)
            std::cout<<std::setw(30)<<std::left<<"HAL: restore   - OK"<<std::endl;
        else
            std::cout<<std::setw(30)<<std::left<<"HAL: restore   - FAIL"<<std::endl;
        std::cout << "========================================" << std::endl;

        reboot(0);
    }else
    {
        std::cout<<std::setw(30)<<std::left<<"HAL: nothing to update - OK"<<std::endl;
        std::cout << "========================================" << std::endl;
    }

}
void UpdateImageOS()
{
    HAL::obj()->updateVerOS();
}
//void setDefaultIP(void)
//{
//    std::vector<TTableNode> listNode;
//    std::string curMac;
//    struct ifreq ifr;
//    int fd;
//    fd = socket(AF_INET,SOCK_DGRAM,0);
//    if(fd == ERROR)
//    {
//        std::cout<<"setDefaultIP(): Didn`t open socket"<<std::endl;
//        return;
//    }
//    ifr.ifr_addr.sa_family = AF_INET;
//    strncpy(ifr.ifr_name,"eth0",IFNAMSIZ-1);
//    int flag = ioctl(fd,SIOCGIFLLADDR,&ifr);
//    if(flag == ERROR)
//    {
//        std::cout<<"setDefaultIP(): Can`t get MAC addres for eth0"<<std::endl;
//        return;
//    }
//    unsigned char* mac = (unsigned char*)ifr.ifr_addr.sa_data;

//    char mas[2];
//    for(int i=0;i<6;i++)
//    {
//        sprintf(mas,"%02X",mac[i]);
//        curMac+=mas;
//        if(i<5)
//            curMac+="-";
//    }


//    printf("%02X:%02X:02X\n",mac[0],mac[1],mac[2]);
//    //! далее нужно пробежаться по файлу и найти соотвествующий ему ip
//    int fdMac = open("/romfs/mac_addr.txt",O_RDONLY,0644);
//    if(fdMac == ERROR)
//    {
//        std::cout<<"setDefaultIP(): Can`t open file - mac_addr.txt"<<std::endl;
//        return;
//    }

//    int sizeFile =  lseek(fdMac,0,SEEK_END);
//    lseek(fdMac,0,SEEK_SET);
//    char *buffer = new char[sizeFile+1];
//    int bytes = read (fdMac,buffer,sizeFile);
//    std::string dataFile;
//    dataFile.append(buffer);
//    //printf("%s\n",dataFile.c_str());

//    std::string::size_type n;
//    n = dataFile.find(";");

//    //! убираем все пробельные символы
//    n = dataFile.find(" ");
//    while(n!=std::string::npos)
//    {
//        dataFile.erase(n,1);
//        n = dataFile.find(" ");
//    };
//    //! убираем переходы на новую строчку
//    n = dataFile.find("\r\n");
//    while(n!=std::string::npos)
//    {
//        dataFile.erase(n,2);
//        n = dataFile.find("\r\n");
//    };

//    n = dataFile.find(";");
//    TTableNode tempNode;
//    while(n!=std::string::npos)
//    {

//        for(int j = 0;j<3;j++)
//        {
//            std::string subStr = dataFile.substr(0,n);
//            dataFile.erase(0,n + 1);

//            n = dataFile.find(";");
//            //i += subStr.size();

//            switch(j)
//    {
//            case 0:{tempNode.nameId = subStr;break;}
//            case 1:{tempNode.mac = subStr;break;}
//            case 2:{tempNode.ip = subStr;break;}
//            };

//    }
//        listNode.push_back(tempNode);

//    };
//    //! ищем данные
//    std::string comIfConfig = "eth0 ";

//    for(int i =0;i<listNode.size();i++)
//    {
//        if(listNode[i].mac == curMac)
//        {
//            //! далее заменить ip на считанный из файла
//            comIfConfig+=listNode[i].ip;
//            ifconfig((char*)(comIfConfig.c_str()));
//            //wdbEndDeviceAddress = IP_TEST;
//            strcpy(ip_node,(char*)(listNode[i].ip.c_str()));
//            //! идентификатор узла
//            nodeId =listNode[i].nameId ;

//            wdbEndDeviceAddress = &(ip_node[0]);
//            printf("set IP");
//        }
//    }

//    close(fd);
//}
