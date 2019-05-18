
#include "vxJNode.h"
#include "hal.h"
#include "chdev.h"
#include "layerRK.h"
VxJNode::VxJNode(uint32_t idClass):ICalculateElement(idClass)
{
    //! получение данных от других узлов
    udpRec = new VxTemplateUdpSocket;
    udpRec->init(HAL::obj()->ethPort(ETH_PORT_JNODE),"");
    udpRec->bindTo();
    udpRec->setBlock(false);

    //! сокет для отправки в другие узлы
    udpSend = new VxTemplateUdpSocket();
    udpSend->init(HAL::obj()->ethPort(ETH_PORT_JNODE),"");

    //! запустить поток для приема данных
    
    
//    switch

//    if(HAL::obj()->typeCurrentNode == E_NODE_CV)
//    {
//        TUdpReciveList *item  = new TUdpReciveList;

//        item->typeNode   = E_NODE_PV;
//        item->idNode     = 1;
//        item->udpSocketRec = new VxTemplateUdpSocket();
//        item->udpSocketRec->init(HAL::obj()->ethPort(ETH_PORT_JNODE),"");

//        listUdpRec.push_back(item);
//    }

//    udpSocket  = new VxTemplateUdpSocket;
//    udpSocket->init(set.portRecive,"");
//    bool res = udpSocket->bindTo();
//    if(res<0)
//    {
//        std::cout<<"VxMPPM: Can`t bind socket"<<std::endl;
//    }


    //! указываем зависимости
    setFreq(ICalculateElement::Hz50);
    setStart();
}
void VxJNode::init()
{
    ICalculateElement::init();
}
void VxJNode::calculate()
{
    //! отправка данных
    sendDataToNodes();
    recDataFromNodes();
}
//! прием данных от узлов
void VxJNode::recDataFromNodes()
{
    int bytes;
    
    do
    {
        memset((void*)&recPacket,0,sizeof(recPacket));
        //! считываем заголовок
        bytes = udpRec->reciveFrom((uint8_t*)&recHead, sizeof(TUdpJNodeHead), MSG_WAITALL | MSG_PEEK);
        if(bytes!=ERROR)
        {
            //! считываем всю запись
            bytes = udpRec->reciveFrom((uint8_t*)&recPacket, (sizeof(TUdpJNodePacket) - (MAX_BUF-recHead.sizeBuf)), MSG_WAITALL);
            if(bytes!=ERROR)
            {
                //! производим разбор данных и запись их в HAL
                parserRecData();
            }
        }
    }while(bytes>0);

}
void VxJNode::parserRecData()
{
    uint16_t offset = 0;
    TUdpJNodeCh *chDes;
    uint8_t *ptr = recPacket.buffer;
    do{
        chDes = (TUdpJNodeCh *)(ptr + offset);
        TCh* ch = HAL::obj()->findCh(chDes->indexCh);
        if(ch==0)
            return;
        if(ch->typeCh == E_CH_RK)
        {
            TDesRK *rk = (TDesRK*)(chDes->data);
            if(ch->setting.ioCh == 1)
            {
                LayerRK::set(ch->setting.numCh,(rk->value));

            }
            memcpy((void*)ch->desData,(void*)rk,sizeof(TDesRK));
            offset += sizeof(TDesRK);
            offset+=sizeof(chDes->indexCh);

        }else if(ch->typeCh == E_CH_AR)
        {
                TDesAr *ar = (TDesAr*)ch->desData;
                ar->active = chDes->data[0];
                //memcpy((void*)ar->active,(void*)chDes->data,sizeof(ar->active));
                //offset +=sizeof(ar->active);
                //chDes = (TUdpJNodeCh *)(ptr + offset);
                memcpy((void*)ar->value,(void*)&(chDes->data[1]),sizeof(uint32_t) * ar->numAddr);
                ch->setting.readyToRead = 1;
                ch->setting.readyToWrite= 1;
                offset += 1;
                offset += (sizeof(uint32_t) * ar->numAddr);
                
                offset+=sizeof(chDes->indexCh);

        }else if(ch->typeCh == E_CH_MIL)
        {
                TDesMIL *mil = (TDesMIL*)ch->desData;
                mil->active = chDes->data[0];
                //memcpy((void*)mil->active,(void*)chDes->data,sizeof(mil->active));
                //offset +=sizeof(mil->active);
                //chDes = (TUdpJNodeCh *)(ptr + offset);
                 memcpy((void*)mil->s.ui16,(void*)chDes->data,32 * sizeof(mil->s.ui16[0]));
               // ch->setting.readyToRead = 1;
               // ch->setting.readyToWrite = 1;
                offset += 1;
                offset += 32 * sizeof(mil->s.ui16[0]);
                offset += sizeof(chDes->indexCh);
        }else if(ch->typeCh == E_IR)
        {
            TDesIR *ir = (TDesIR*)(chDes->data);
            memcpy((void*)ch->desData,(void*)ir,sizeof(TDesIR));
            offset += sizeof(TDesIR);
            offset+=sizeof(chDes->indexCh);
        }else if(ch->typeCh == E_CH_DAC)
        {
            TDesDAC *dac = (TDesDAC*)(chDes->data);
            memcpy((void*)ch->desData,(void*)dac,sizeof(TDesDAC));         
            offset += sizeof(TDesDAC);
            offset+=sizeof(chDes->indexCh);
        }else if(ch->typeCh == E_GEN_U)
        {
            TDesGen_U *gen_u = (TDesGen_U*)(chDes->data);
            memcpy((void*)ch->desData,(void*)gen_u,sizeof(TDesGen_U));
            offset += sizeof(TDesGen_U);
            offset+=sizeof(chDes->indexCh);
        }else if(ch->typeCh == E_GEN_NU)
        {
            TDesGen_NU *gen_nu = (TDesGen_NU*)(chDes->data);
            memcpy((void*)ch->desData,(void*)gen_nu,sizeof(TDesGen_NU));
            offset += sizeof(TDesGen_NU);
            offset+=sizeof(chDes->indexCh);
        }else if(ch->typeCh == E_ITP)
        {
            TDesITP *itp = (TDesITP*)(chDes->data);
            memcpy((void*)ch->desData,(void*)itp,sizeof(TDesITP));
            offset += sizeof(TDesITP);
            offset+=sizeof(chDes->indexCh);
        }
            
        
        
    }while(offset<=recPacket.head.sizeBuf);
}

void VxJNode::packetFull(uint16_t *offset, unsigned long ip)
{
    sendPacket.head.type = 0;
    sendPacket.head.sizeBuf = *offset;
    udpSend->sendTo((uint8_t*)&sendPacket,
            sizeof(TUdpJNodeHead) + *offset,
            HAL::obj()->ethPort(ETH_PORT_JNODE),
            ip);
    *offset = 0;
}
//! отправка данных в другие узлы
void VxJNode::sendDataToNodes()
{
    uint16_t offset = 0;
    
    //int bytes = 0;
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
        
        TTableChHAL *table = &(HAL::obj()->chTable);
        //! пробегаем по таблице с параметрами
        for(int j = 0;j<table->numCh;j++)
        {
            TCh *ch = &(HAL::obj()->chTable.ch[j]);
           
//            //! выполняем отправку данных
//            if((offset>=MAX_LIM_BUF) || ((offset+sizeof(TDesAr))>=MAX_LIM_BUF))
//            {
//               packetFull(&offset,inet_addr(eth1->ip));
//               sendPacket.head.type = 0;
//               sendPacket.head.sizeBuf = offset;
//               bytes = udpSend->sendTo((uint8_t*)&sendPacket,
//                                        sizeof(TUdpJNodeHead) + offset,
//                                        HAL::obj()->ethPort(ETH_PORT_JNODE),
//                                        inet_addr(eth1->ip));
//               offset = 0;
//            }      
            if((ch->typeNode == eth->toTypeNode &&
               ch->idNode   == eth->toIdNode &&                
               ch->setting.readyToWrite == 1) || 
               (ch->typeNode == HAL::obj()->typeCurrentNode &&
                ch->idNode == HAL::obj()->currentNode &&
               ch->setting.ioCh==0))
            {
                if(offset >=MAX_LIM_BUF)
                    packetFull(&offset,inet_addr(eth1->ip));
                
                TUdpJNodeCh *chPacket = (TUdpJNodeCh *)(sendPacket.buffer + offset);
                chPacket->indexCh = j;                
                //проверяем признак подготовленных данных
                //добавляем их в пакет, пока не достиги максимума
                if(ch->typeCh == E_CH_AR)//тип канала ДПК
                {                     
                    //! указатель на ДПК
                    TDesAr *ar = (TDesAr*)ch->desData;    
                    
                    if((offset + (sizeof(uint32_t) * ar->numAddr)) >=MAX_LIM_BUF)
                        packetFull(&offset,inet_addr(eth1->ip));
                    
                    chPacket = (TUdpJNodeCh *)(sendPacket.buffer + offset);
                    chPacket->indexCh = j;
//                    if((offset + sizeof(TDesAr)) >=MAX_BUF)
//                        packetFull(&offset,inet_addr(eth1->ip));
                       
                    //if(ch->setting.numCh == 1)
                    //{
                        //! копирование данных
                        chPacket->data[0] = ar->active;
                        
                        /*memcpy((void*)(chPacket->data), (void*)ar->active,sizeof(ar->active));
                        offset +=sizeof(ar->active);
                        chPacket = (TUdpJNodeCh *)(sendPacket.buffer + offset);*/
                        memcpy((void*)(&(chPacket->data[1])), (void*)ar->value, sizeof(uint32_t) * ar->numAddr);
                        //! смещение относительно начала буфера
                        offset += 1;
                        offset += (sizeof(uint32_t) * ar->numAddr);
                        offset +=  sizeof(chPacket->indexCh);
                        //offset +=  sizeof(uint8_t);
                        //offset += (sizeof(chPacket) + sizeof(uint32_t) * ar->numAddr);
                    //}
                }else if(ch->typeCh == E_CH_MIL)
                {
                    //! указатель на МКИО
                    TDesMIL *mil = (TDesMIL*)ch->desData;   
                    if((offset + (sizeof(mil->s))) >=MAX_LIM_BUF)
                        packetFull(&offset,inet_addr(eth1->ip));
                    chPacket = (TUdpJNodeCh *)(sendPacket.buffer + offset);
                    chPacket->indexCh = j;
                    chPacket->data[0] = mil->active;
                    //memcpy((void*)(chPacket->data), (void*)mil->active,sizeof(mil->active));
                    offset +=1;
                    //chPacket = (TUdpJNodeCh *)(sendPacket.buffer + offset);
                    
                    //! копирование данных
                    memcpy((void*)(&(chPacket->data[1])), (void*)mil->s.ui16, 32 * sizeof(mil->s.ui16[0]));
                    //! смещение относительно начала буфера
                    offset += sizeof(mil->s);
                    offset += sizeof(chPacket->indexCh);
                    
                }else if(ch->typeCh == E_CH_RK) //тип канала РК                
                {                    
                    //! указатель на РК 
                    TDesRK *rk = (TDesRK*)ch->desData;    
                    //! копирование данных
                    memcpy((void*)(chPacket->data), (void*)rk, sizeof(TDesRK));
                    //! смещение относительно начала буфера
                    offset +=sizeof(TDesRK);
                    offset +=sizeof(chPacket->indexCh);
                }else if(ch->typeCh == E_IR)
                {
                    //! указатель на ИС 
                    TDesIR *ir = (TDesIR*)ch->desData;    
                    //! копирование данных
                    memcpy((void*)(chPacket->data), (void*)ir, sizeof(TDesIR));
                    //! смещение относительно начала буфера
                    offset +=sizeof(TDesIR);
                    offset +=sizeof(chPacket->indexCh);
                }else if(ch->typeCh == E_CH_DAC)
                {
                    //! указатель на ИС 
                    TDesDAC *dac = (TDesDAC*)ch->desData;
                    //! копирование данных
                    memcpy((void*)(chPacket->data), (void*)dac, sizeof(TDesDAC));
                    //! смещение относительно начала буфера
                    offset +=sizeof(TDesDAC);
                    offset +=sizeof(chPacket->indexCh);
                }else if(ch->typeCh == E_GEN_U)
                {
                    //! указатель на генератор
                    TDesGen_U *gen_u = (TDesGen_U*)ch->desData;
                    //! копирование данных
                    memcpy((void*)(chPacket->data), (void*)gen_u, sizeof(TDesGen_U));
                    //! смещение относительно начала буфера
                    offset +=sizeof(TDesGen_U);
                    offset +=sizeof(chPacket->indexCh);
                }else if(ch->typeCh == E_GEN_NU)
                {
                    TDesGen_NU *gen_nu = (TDesGen_NU*)ch->desData;
                    //! копирование данных
                    memcpy((void*)(chPacket->data), (void*)gen_nu, sizeof(TDesGen_NU));
                    //! смещение относительно начала буфера
                    offset +=sizeof(TDesGen_NU);
                    offset +=sizeof(chPacket->indexCh);
                }else if(ch->typeCh == E_ITP)
                {
                    TDesITP *itp = (TDesITP*)ch->desData;
                    //! копирование данных
                    memcpy((void*)(chPacket->data), (void*)itp, sizeof(TDesITP));
                    //! смещение относительно начала буфера
                    offset +=sizeof(TDesITP);
                    offset +=sizeof(chPacket->indexCh);                    
                }
            }               
        }
        if(offset!=0)
        {
            packetFull(&offset,inet_addr(eth1->ip));            
        }      
    }

}
