/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Universita' degli Studi di Napoli Federico II
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/log.h"
#include "rr-ofdma-manager.h"
#include "wifi-ack-policy-selector.h"
#include "wifi-phy.h"
#include <utility>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RrOfdmaManager");

NS_OBJECT_ENSURE_REGISTERED (RrOfdmaManager);

TypeId
RrOfdmaManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RrOfdmaManager")
    .SetParent<OfdmaManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RrOfdmaManager> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in the MU DL OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&RrOfdmaManager::m_nStations),
                   MakeUintegerChecker<uint8_t> (1, 74))
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_OFDMA even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RrOfdmaManager::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_OFDMA if DL_OFDMA was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrOfdmaManager::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The size in bytes of the solicited PSDU (to be sent in an HE TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&RrOfdmaManager::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ChannelBw",
                   "For TESTING only",
                   UintegerValue (20),
                   MakeUintegerAccessor (&RrOfdmaManager::m_bw),
                   MakeUintegerChecker<uint16_t> (5, 160))
  ;
  return tid;
}

RrOfdmaManager::RrOfdmaManager ()
  : m_startStation (0)
{
  NS_LOG_FUNCTION (this);
}

RrOfdmaManager::~RrOfdmaManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}


 /**
   * Compute the TX vector and the TX params for a DL MU transmission assuming
   * the given list of receiver stations, the given RU type and the given type
   * of acknowledgment sequence.
   *
   * \param staList the list of receiver stations for the DL MU transmission
   * \param ruType the RU type
   * \param dlMuAckSequence the ack sequence type
   */

void
RrOfdmaManager::InitTxVectorAndParams (std::map<Mac48Address, DlPerStaInfo> staList,
                                         HeRu::RuType ruType, DlMuAckSequenceType dlMuAckSequence)
{
  std::cout<<"\n start: InitTxVectorAndParams";
  NS_LOG_FUNCTION (this);
//NS_LOG_FUNCTION("ru:::::::::"<<ruType);
  m_txVector = WifiTxVector ();
  m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
  m_txVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
  m_txVector.SetTxPowerLevel (GetWifiRemoteStationManager ()->GetDefaultTxPowerLevel ());
  m_txParams = MacLowTransmissionParameters ();
  m_txParams.SetDlMuAckSequenceType (dlMuAckSequence);


  Ptr<WifiMacQueueItem> mpdu = Copy (m_mpdu);

std::cout<<"\n inside InitTxVectorAndParams: before for loop\n";

/////////////////////////////////////////Reshan
unsigned int i=0; 
  for (auto& sta : staList)
    {
      mpdu->GetHeader ().SetAddr1 (sta.first);
      // Get the TX vector used to transmit single user frames to the receiver
      // station (the RU index will be assigned by ComputeDlOfdmaInfo)
      WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu);
      NS_LOG_DEBUG ("Adding STA with AID=" << sta.second.aid << " and TX mode="
                    << suTxVector.GetMode () << " to the TX vector");


///////ReshanFaraz///////////////////
std::cout<<"\n inside InitTxVectorAndParams: before your code\n";

if(finalRUAlloc.size()!=0){
  std::cout<<"mappedRuAllocated-----"<<mappedRuAllocated[i];
  m_txVector.SetHeMuUserInfo (sta.second.aid, {{false,mappedRuAllocated[i], 1}, suTxVector.GetMode (), suTxVector.GetNss ()});
  i++;
}
else      
  m_txVector.SetHeMuUserInfo (sta.second.aid, {{false, ruType, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});
//////////////////////////////

      // Add the receiver station to the appropriate list of the TX params
      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (sta.second.tid)];
      BlockAckReqType barType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                                ? txop->GetBlockAckReqType (sta.first, sta.second.tid)
                                : BlockAckReqType::COMPRESSED;
      BlockAckType baType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                            ? txop->GetBlockAckType (sta.first, sta.second.tid)
                            : BlockAckType::COMPRESSED;

      if (dlMuAckSequence == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          // Enable BAR/BA exchange for all the receiver stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR)
        {
          // Send a MU-BAR to all the stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Expect to receive a Block Ack from all the stations
          m_txParams.EnableBlockAck (sta.first, baType);
        }
    }
}





/**
   * Select the format of the next transmission, assuming that the AP gained
   * access to the channel to transmit the given MPDU.
   *
   * \param mpdu the MPDU the AP intends to transmit
   * \return the format of the next transmission
   */

OfdmaTxFormat
RrOfdmaManager::SelectTxFormat (Ptr<const WifiMacQueueItem> mpdu)
{
  // --- for TESTING only ---
//   for (uint8_t i = 1; i <= m_nStations; i++)
//     {
//       DlPerStaInfo info {i, 0};
//       m_staInfo.push_back (std::make_pair (Mac48Address::Allocate (), info));
//     }
//   return OfdmaTxFormat::DL_OFDMA;
  // --- --- ---
  NS_LOG_FUNCTION (this << *mpdu);
  NS_ASSERT (mpdu->GetHeader ().IsQosData ());



  if (m_enableUlOfdma && GetTxFormat () == DL_OFDMA)
    {
      // check if an UL OFDMA transmission is possible after a DL OFDMA transmission
      NS_ABORT_MSG_IF (m_ulPsduSize == 0, "The UlPsduSize attribute must be set to a non-null value");

      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (mpdu->GetHeader ().GetQosTid ())];
      m_ulMuAckSequence = txop->GetAckPolicySelector ()->GetAckSequenceForUlMu ();
      MacLowTransmissionParameters params;
      params.SetUlMuAckSequenceType (m_ulMuAckSequence);
      BlockAckType baType;

      if (m_ulMuAckSequence == UL_MULTI_STA_BLOCK_ACK)
        {
          baType = BlockAckType::MULTI_STA;
          for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
            {
              auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
              if (addressIt != m_apMac->GetStaList ().end ())
                {
                  baType.m_bitmapLen.push_back (32);
                  params.EnableBlockAck (addressIt->second, baType);
                }
              else
                {
                  NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
                }
            }
        }
      else
        {
          NS_FATAL_ERROR ("Sending Block Acks in an MU DL PPDU is not supported yet");
        }

      CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);

      // compute the maximum amount of time that can be granted to stations.
      // This value is limited by the max PPDU duration
      Time maxDuration = GetPpduMaxTime (m_txVector.GetPreambleType ());

      // compute the time required by stations based on the buffer status reports, if any
      uint32_t maxBufferSize = 0;

      for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
        {
          auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
          if (addressIt != m_apMac->GetStaList ().end ())
            {
              uint8_t queueSize = m_apMac->GetMaxBufferStatus (addressIt->second);
              if (queueSize == 255)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is unknown");
                  maxBufferSize = std::max (maxBufferSize, m_ulPsduSize);
                }
              else if (queueSize == 254)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is not limited");
                  maxBufferSize = 0xffffffff;
                  break;
                }
              else
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is " << +queueSize);
                  maxBufferSize = std::max (maxBufferSize, static_cast<uint32_t> (queueSize * 256));
                }
            }
          else
            {
              NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
            }
        }

      // if the maximum buffer size is 0, skip UL OFDMA and proceed with trying DL OFDMA
      if (maxBufferSize > 0)
        {
          // if we are within a TXOP, we have to consider the response time and the
          // remaining TXOP duration
          if (txop->GetTxopLimit ().IsStrictlyPositive ())
            {
              // we need to define the HE TB (trigger based) PPDU duration in order to compute the response to
              // the Trigger Frame. Let's use 1 ms for this purpose. We'll subtract it later.
              uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (MilliSeconds (1),
                                                                              m_low->GetPhy ()->GetFrequency ());
              trigger.SetUlLength (length);

              Ptr<Packet> packet = Create<Packet> ();
              packet->AddHeader (trigger);
              WifiMacHeader hdr;
              hdr.SetType (WIFI_MAC_CTL_TRIGGER);
              hdr.SetAddr1 (Mac48Address::GetBroadcast ());
              Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, hdr);

              Time response = m_low->GetResponseDuration (params, m_txVector, item);

              // Add the time to transmit the Trigger Frame itself
              WifiTxVector txVector = GetWifiRemoteStationManager ()->GetRtsTxVector (hdr.GetAddr1 (), &hdr, packet);

              response += m_low->GetPhy ()->CalculateTxDuration (item->GetSize (), txVector,
                                                                 m_low->GetPhy ()->GetFrequency ());

              // Subtract the duration of the HE TB PPDU
              response -= WifiPhy::ConvertLSigLengthToHeTbPpduDuration (length, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency ());

              if (response > txop->GetTxopRemaining ())
                {
                  // an UL OFDMA transmission is not possible. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
                  m_staInfo.clear ();
                  return DL_OFDMA;
                }

              maxDuration = Min (maxDuration, txop->GetTxopRemaining () - response);
            }

          Time bufferTxTime = m_low->GetPhy ()->CalculateTxDuration (maxBufferSize, m_txVector,
                                                                     m_low->GetPhy ()->GetFrequency (),
                                                                     trigger.begin ()->GetAid12 ());
          if (bufferTxTime < maxDuration)
            {
              // the maximum buffer size can be transmitted within the allowed time
              maxDuration = bufferTxTime;
            }
          else
            {
              // maxDuration may be a too short time. If it does not allow to transmit
              // at least m_ulPsduSize bytes, give up the UL MU transmission for now
              Time minDuration = m_low->GetPhy ()->CalculateTxDuration (m_ulPsduSize, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency (),
                                                                        trigger.begin ()->GetAid12 ());
              if (maxDuration < minDuration)
                {
                  // maxDuration is a too short time. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Available time " << maxDuration << " is too short");
                  m_staInfo.clear ();
                  return DL_OFDMA;
                }
            }

          // maxDuration is the time to grant to the stations. Store it in the TX vector
          NS_LOG_DEBUG ("HE TB PPDU duration: " << maxDuration.ToDouble (Time::MS));
          uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (maxDuration,
                                                                          m_low->GetPhy ()->GetFrequency ());
          m_txVector.SetLength (length);
          m_txParams = params;
          return UL_OFDMA;
        }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // get the list of associated stations ((AID, MAC address) pairs)
  const std::map<uint16_t, Mac48Address>& staList = m_apMac->GetStaList ();
  auto startIt = staList.find (m_startStation);

std::cout<<"SelectTxFormat: sta list size"<<staList.size()<<"\n";
std::cout<<"SelectTxFormat: start station"<<m_startStation<<"\n";
std::cout<<"SelectTxFormat: total station"<<m_nStations<<"\n";

  // This may be the first invocation or the starting station left
  if (startIt == staList.end ())
    {
      startIt = staList.begin ();
      m_startStation = startIt->first;
std::cout<<"SelectTxFormat: if (startIt == staList.end ()),  start station"<<m_startStation<<"\n";

    }


  uint8_t currTid = mpdu->GetHeader ().GetQosTid ();
  AcIndex primaryAc = QosUtilsMapTidToAc (currTid);
  m_staInfo.clear ();

  // If the primary AC holds a TXOP, we can select a station as a receiver of
  // the MU PPDU only if the AP has frames to send to such station that fit into
  // the remaining TXOP time. To this end, we need to determine the type of ack
  // sequence and the time it takes. To compute the latter, we can call the
  // MacLow::GetResponseDuration () method, which requires TX vector and TX params.
  // Our best guess at this stage is that the AP has frames to send to all the
  // associated stations and hence we initialize the TX vector and the TX params
  // by considering the starting station and those that immediately follow it in
  // the list of associated stations.


  std::size_t count = m_nStations;


  HeRu::RuType ruType = GetNumberAndTypeOfRus (m_low->GetPhy ()->GetChannelWidth (), count);



  NS_ASSERT (count >= 1);

  std::map<Mac48Address, DlPerStaInfo> guess;
  auto staIt = startIt;
  do
    {
      guess[staIt->second] = {staIt->first, currTid};
      if (++staIt == staList.end ())
        {
          staIt = staList.begin ();
        }
    } while (guess.size () < count && staIt != startIt);


  Ptr<WifiAckPolicySelector> ackSelector = m_qosTxop[primaryAc]->GetAckPolicySelector ();
  NS_ASSERT (ackSelector != 0);
  m_dlMuAckSequence = ackSelector->GetAckSequenceForDlMu ();
  
//This call doesn't matter for our use case because this function is called again from compute dl ofdma which again finds the tx vector and params
InitTxVectorAndParams (guess, ruType, m_dlMuAckSequence);

  // if the AC owns a TXOP, compute the time available for the transmission of data frames
  Time txopLimit = Seconds (0);
  if (m_qosTxop[primaryAc]->GetTxopLimit ().IsStrictlyPositive ())
    {
      // TODO Account for MU-RTS/CTS when implemented
      CtrlTriggerHeader trigger;

      if (m_dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR
          || m_dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Need to prepare the MU-BAR to correctly get the response time
          trigger = GetTriggerFrameHeader (m_txVector, 5);
          trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (trigger, m_txParams));
        }
      txopLimit = m_qosTxop[primaryAc]->GetTxopRemaining () - GetResponseDuration (m_txParams, m_txVector, trigger);

      if (txopLimit.IsNegative ())
        {
          if (m_forceDlOfdma)
            {
              NS_LOG_DEBUG ("Not enough TXOP remaining time: return DL_OFDMA with empty set of receiver stations");
              return OfdmaTxFormat::DL_OFDMA;
            }
          NS_LOG_DEBUG ("Not enough TXOP remaining time: return NON_OFDMA");
          return OfdmaTxFormat::NON_OFDMA;
        }
    }


/////////////////////////////////////////////////////////////////
dataStaPair1.clear(); 
v_QosType.clear();
v_powerLevel.clear();
v_dataStaPair.clear();
finalStaPairIndex.clear();


  do
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << startIt->second << ", AID=" << startIt->first << ")");
      // check if the AP has at least one frame to be sent to the current station
      for (uint8_t tid : std::initializer_list<uint8_t> {currTid, 1, 2, 0, 3, 4, 5, 6, 7})
        {
          AcIndex ac = QosUtilsMapTidToAc (tid);
          // check that a BA agreement is established with the receiver for the
          // considered TID, since ack sequences for DL MU PPDUs require block ack
          if (ac >= primaryAc && m_qosTxop[ac]->GetBaAgreementEstablished (startIt->second, tid))
            {
              mpdu = m_qosTxop[ac]->PeekNextFrame (tid, startIt->second);

              // we only check if the first frame of the current TID meets the size
              // and duration constraints. We do not explore the queues further.
              if (mpdu != 0)
                {
                  // Use a temporary TX vector including only the STA-ID of the
                  // candidate station to check if the MPDU meets the size and time limits.
                  // An RU of the computed size is tentatively assigned to the candidate
                  // station, so that the TX duration can be correctly computed.
                  WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu),
                               muTxVector;

                  muTxVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
                  muTxVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
                  muTxVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
                  muTxVector.SetHeMuUserInfo (startIt->first,
                                              {{false, ruType, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});

                  if (m_low->IsWithinSizeAndTimeLimits (mpdu, muTxVector, 0, txopLimit))
                    {

                      int type_Qos;
                      if(ac==AC_VO){
                        type_Qos=1;
                      }
                      else if(ac==AC_BE)
                      {
                        type_Qos=3;
                      }
                      else if(ac==AC_VI)
                      {
                        type_Qos=2;
                      }
                      else{
                        type_Qos=4;
                      }

                      double txpowerstart=powww.GetTxPowerStart();

                      v_QosType.push_back(type_Qos);
                      v_powerLevel.push_back(txpowerstart);
                       v_dataStaPair.push_back(mpdu->GetPacket()->GetSize());

  //                      for(unsigned int i=0;i<v_dataStaPair.size();i++){
  // std::cout<<"\n\n vector data of station"<<v_dataStaPair[i];
//}













                      // the frame meets the constraints, add the station to the list
                      NS_LOG_DEBUG ("Adding candidate STA (MAC=" << startIt->second << ", AID="
                                    << startIt->first << ") TID=" << +tid);
                      DlPerStaInfo info {startIt->first, tid};
                      m_staInfo.push_back (std::make_pair (startIt->second, info));
                      break;    // terminate the for loop
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("No frames to send to " << startIt->second << " with TID=" << +tid);
                }
            }
        }

      // move to the next station in the map
      startIt++;
      if (startIt == staList.end ())
        {
          startIt = staList.begin ();
        }
    } while (m_staInfo.size () < m_nStations && startIt->first != m_startStation);

  if (m_staInfo.empty ())
    {
      if (m_forceDlOfdma)
        {
          NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return DL_OFDMA with empty set of receiver stations");
          return OfdmaTxFormat::DL_OFDMA;
        }
      NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return NON_OFDMA");
      return OfdmaTxFormat::NON_OFDMA;
    }



for(unsigned int i=0;i<v_dataStaPair.size();i++){
  std::cout<<"\n\n vector data of station"<<v_dataStaPair[i];
}



  m_startStation = startIt->first;
  return OfdmaTxFormat::DL_OFDMA;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 /**
   * Given the channel bandwidth and the number of stations candidate for being
   * assigned an RU, maximize the number of candidate stations that can be assigned
   * an RU subject to the constraint that all the stations must be assigned an RU
   * of the same size (in terms of number of tones).
   *
   * \param bandwidth the channel bandwidth in MHz
   * \param nStations the number of candidate stations. On return, it is set to
   *                  the number of stations that are assigned an RU
   * \return the RU type
   */

HeRu::RuType
RrOfdmaManager::GetNumberAndTypeOfRus (uint16_t bandwidth, std::size_t& nStations) const
{
  NS_LOG_FUNCTION (this);

NS_LOG_FUNCTION("no.of sta, count value:::::::::::::::::::::::::::::::::::::;; "<<nStations);
  HeRu::RuType ruType;
  uint8_t nRusAssigned = 0;

  // iterate over all the available RU types
  for (auto& ru : HeRu::m_heRuSubcarrierGroups)
    {
      if (ru.first.first == bandwidth && ru.second.size () <= nStations)
        {
          ruType = ru.first.second;
          nRusAssigned = ru.second.size ();
          break;
        }
      else if (bandwidth == 160 && ru.first.first == 80 && (2 * ru.second.size () <= nStations))
        {
          ruType = ru.first.second;
          nRusAssigned = 2 * ru.second.size ();
          break;
        }
    }
  if (nRusAssigned == 0)
    {
      NS_ASSERT (bandwidth == 160 && nStations == 1);
      nRusAssigned = 1;
      ruType = HeRu::RU_2x996_TONE;
    }

NS_LOG_FUNCTION("ru type---------------------: "<<ruType);
  nStations = nRusAssigned;
  return ruType;
}



double RrOfdmaManager::calculate_a_i(int type_of_Application)
{
  std::cout<<"\ncalculating Type of Application";
  double a_i;
  switch(type_of_Application){
    case 1:
    a_i=16;
    //a_i=-(log10(10)/100);
    break;

    case 2:
    a_i=8;
    //a_i=-(log10(20)/150);
    break;

    case 3:
    a_i=4;
    //a_i=-(log10(20)/300);
    break;

    case 4:
    a_i=2;
    //a_i=-(log10(20)/400);
    break;
    default:
    break;
  }
  return a_i;
    

}
double RrOfdmaManager::averageChannelcapacity()
{
  std::cout<<"\ncalculating average channel capacity";
  // auto a=powerLevel.begin();
  double avg_channel_capacity;
  // while(a!=powerLevel.end())
  // {
  //     avg_channel_capacity+= a->first;
  //     a++;
  // }

for(unsigned int i = 0; i<v_powerLevel.size();i++){
avg_channel_capacity+= v_powerLevel[i];
  //     

}

  return (avg_channel_capacity/v_powerLevel.size());
}

WifiSpectrumBand
RrOfdmaManager::GetBand (uint16_t bandWidth, uint8_t /*bandIndex*/)
{
  WifiSpectrumBand band;
  band.first = 0;
  band.second = 0;
  return band;
}



void RrOfdmaManager::RuAlloc(int number_of_clients)
{

  std::cout<<"\n Inside RU Alloca\n "<<number_of_clients;
  //std::vector<int> RuSet;

  if(number_of_clients>0){
switch(number_of_clients){
  case 1:
  finalRUAlloc.push_back(242);
  break;

  case 2:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(106);
 // RuSet.push_back(26);
  break;

  case 3:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(26);
  break;

  case 4:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  break;

  case 5:
  finalRUAlloc.push_back(106);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
 break;

case 6:
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26); // other combination are also there
  break;

  case 7:
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;

  case 8:
  finalRUAlloc.push_back(52);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  finalRUAlloc.push_back(26);
  break;


  default:
 for(int i=0;i<9;i++){
  finalRUAlloc.push_back(26);
 }
break;

}

}
else{
  std::cout<<"No client to allocate Resources";
  

}
}













void RrOfdmaManager::Largest_Weighted_First(){


int lower=1,upper=11;
int count=v_dataStaPair.size();

  random_MCS.clear();
for(int i=0;i<count;i++){
  int num= (rand()%(upper-lower+1))+lower;
  random_MCS.push_back(num);

}


  std::cout<<"inside Largest_Weighted_First before clearing vectors";
// check for bandwidth if different bandwidh and genealization of Ru allocation used
// auto a=dataStaPair.begin();
// auto b=powerLevel.begin();
// auto c=QosType.begin();
//auto d=dataStaPair1.begin();
//auto e=finalStaPairIndex.begin();
//a++;
int countt=0;


 std::cout<<"inside Largest_Weighted_First before calculating (loop)";

// while(a!=dataStaPair.end() || b!=powerLevel.end() || c!=QosType.end()){
//   //get type of application and calucate what is the  Pr {W i > T i } £ d i
  
//   double a_i_i=calculate_a_i(c->first) ;   ////=log(delta)/Time
//   double avg_chn_cap=averageChannelcapacity();
//   //         ///maximize= (-log(detla)*Data*Transimmison power)/averagePower
//   double final_cost=((a->first*a_i_i*b->first)/avg_chn_cap);

//   std::cout<<"\n Before dataStaPair1";
//  dataStaPair1.push_back(std::make_pair(final_cost,a->second));
//  std::cout<<"\n Before finalStaPairIndex-----datastapair alur"<<final_cost;
//    //finalStaPairIndex.push_back(std::make_pair(final_cost,countt));
//   //std::cout<<" \n \n Inside dataStaPair1 First"<<d->first<<"  TID  "<<d->second;
//   //std::cout<<" \n \n (While )Inside Final Sta Pair First----"<<finalStaPairIndex[countt].first<<"  TID  "<<finalStaPairIndex[countt].second;

// std::cout<<"\n a++";
// a++;
// std::cout<<"\n B++";
// b++;
// std::cout<<"\n c++";
// c++;
// //d++;



// }
//double avg_chn_cap=averageChannelcapacity();

//int j=0,k=0;
for(unsigned int i = 0; i<v_dataStaPair.size();i++){
  //get type of application and calucate what is the  Pr {W i > T i } £ d i
  
  double a_i_i=calculate_a_i(v_QosType[i]) ;   ////=log(delta)/Time
  

//   std::cout<<"\n\n\n Average Channel Capacity---------------------"<<avg_chn_cap;


//   std::cout<<"\n\n\n Probbilty Model ---------------------"<<a_i_i;

//   std::cout<<"\n\n\n Power of Ith stations---------------------"<<v_powerLevel[i];

//   std::cout<<"\n\n\n Data of Ith stations ---------------------"<<v_dataStaPair[i];
//   //         ///maximize= (-log(detla)*Data*Transimmison power)/averagePower


//   if(avg_chn_cap==0){
//     std::cout<<"\n inside avg_chn_cap==0";
//     if(v_powerLevel[i]==0){
//       std::cout<<"\n inside avg_chn_cap==0  && powerLevel==0";

//       final_cost=((v_dataStaPair[i]*a_i_i*5.0)/2.0);
//       std::cout<<"fianl cost"<<final_cost; 
//     }
//  //final_cost=((v_dataStaPair[i]*a_i_i*v_powerLevel[i])/2.0);
// }


// else{
//   std::cout<<"\n else inside avg_chn_cap not 0";

//     if(v_powerLevel[i]==0){
//        std::cout<<"\n inside avg_chn_cap==0  && powerLevel==0";

//       final_cost=((v_dataStaPair[i]*a_i_i*5)/avg_chn_cap);
//       std::cout<<"fianl cost"<<final_cost; 
//     }
     
//   final_cost=((v_dataStaPair[i]*a_i_i*v_powerLevel[i])/avg_chn_cap);


  
//}
  final_cost=((v_dataStaPair[i]*a_i_i*5.0)/2.0);
  std::cout<<"\n\n\n Final Cost---------------------"<<final_cost;


  std::cout<<"\n Before dataStaPair1 inserting";
 dataStaPair1.push_back(std::make_pair(final_cost,2));
 //std::cout<<"\n Before finalStaPairIndex-----datastapair alur"<<final_cost;
   //finalStaPairIndex.push_back(std::make_pair(final_cost,countt));
  //std::cout<<" \n \n Inside dataStaPair1 First"<<d->first<<"  TID  "<<d->second;
  //std::cout<<" \n \n (While )Inside Final Sta Pair First----"<<finalStaPairIndex[countt].first<<"  TID  "<<finalStaPairIndex[countt].second;

// std::cout<<"\n a++";
// j++;
// std::cout<<"\n B++";
// k++;
// std::cout<<"\n c++";
// c++;
//d++;
}














std::cout<<"\n After While Loop in LWDF";
//auto x=dataStaPair1.begin();
std::cout<<"\n Before dataStaPair1 Loop fofr finalStaPairIndex";
int zz = dataStaPair1.size();
std::cout<<"\n"<<zz;
int i=0;

for(int x=0;x<zz;x++){

  std::cout<<"\n  \n dataStaPair1 data ------"<<dataStaPair1[x].first<<"------ Second"<<dataStaPair1[x].second;
}

finalStaPairIndex.clear();
for(int x=0;x<zz;x++)
{
  std::cout<<"\n Inside single finalStaPairIndex";

  std::cout<<"\n  \n data to be inserted finalStaPairIndexnal------"<<dataStaPair1[x].first<<"count------value"<<countt;
  finalStaPairIndex.push_back(std::make_pair(dataStaPair1[x].first,countt));
  //zz--;
  i++;
  countt++;
}

// for(int x=0;x<zz;x++){

//   std::cout<<"\n  \n finalStaPairIndex data  ------"<<finalStaPairIndex[x].first<<"------ index-------"<<finalStaPairIndex[x].second;
// }


//void insertionSort(int arr[], int n)  
//{  
//{  
std::cout<<"\n \n Before Sorting";
int n=finalStaPairIndex.size();
std::cout<<"\n finalStaPairIndex Size"<<n;
    int counter, key, counter1;  
    for (counter = 1; counter < n; counter++) 
    {  
        key = finalStaPairIndex[counter].first;  
        counter1 = counter - 1;  
        std::cout<<"\n \n inside insertion sort";
        /* Move elements of arr[0..i-s1], that are  
        greater than key, to one position ahead  
        of their current position */
        while (counter1 >= 0 && finalStaPairIndex[counter1].first < key) 
        {  
            finalStaPairIndex[counter1+1].first = finalStaPairIndex[counter].first;  
            counter1 = counter1 - 1;  
        }  
        finalStaPairIndex[counter+1].first = key;  
    }  


std::cout<<"\n \n After  Sorting";

int xyz=finalStaPairIndex.size();
for(int x=0;x<xyz;x++){

  std::cout<<"\n  \n finalStaPairIndex data  ------"<<finalStaPairIndex[x].first<<"------ index-------"<<finalStaPairIndex[x].second;
}



//sort(dataStaPair1.end(),dataStaPair1.begin());

//sort(finalStaPairIndex.end(),finalStaPairIndex.begin());



//

//

//
// 20 Mhz channel , may geralized later
//std::vector<int> finalRUAlloc;





int noOfSTA=finalStaPairIndex.size();
int temp2=noOfSTA;

std::cout<<"\n \n Before RuAlloc Call ";
RuAlloc(noOfSTA); 
for(unsigned int j=0;j<finalRUAlloc.size();j++){
  std::cout<<"\n--- finalRUAlloc   "<<finalRUAlloc[j];
}
mappedRuAllocated.clear();
std::cout<<"\n \n Before Mapped RU LOOP Call ";
for (int i=0; i<noOfSTA; i++){
 

switch(finalRUAlloc[i]){
  case 26: mappedRuAllocated.push_back(HeRu::RU_26_TONE);
           break;
  case 52: mappedRuAllocated.push_back(HeRu::RU_52_TONE);
           break;
  case 106: mappedRuAllocated.push_back(HeRu::RU_106_TONE);
           break;
  case 242: mappedRuAllocated.push_back(HeRu::RU_242_TONE);
           break;
  case 484: mappedRuAllocated.push_back(HeRu::RU_484_TONE);
           break;
  }
  std::cout<<"Mapping============================="<<mappedRuAllocated[i]<<"\n";
}

std::cout<<"\nmapping done\n";

//bestRuConfigTempVar.clear();
//bestAssignment.clear();




//finalRUAlloc.clear();
// finalRUAlloc=RuAlloc(noOfSTA);  /// Allocation Logic implemented in RUalloc // finalRuAlloc Example= {106,106,26}
// std::cout<<"\n \n Before finalRUAlloc ";
// mapped_He_RU(finalRUAlloc);


//Sta Allocated//////////////
// staAllocated.clear();
// std::cout<<"\n \n Before STAPUSH BACK ";
// auto startIt = m_staInfo.begin();
//  for (int i=0; i<noOfSTA; i++){
//             staAllocated.push_back (*startIt);
//           startIt++;
//         }
       // std::cout<<"\n \n Before Sorting";

//STA WIth Decreasing order.
//std::cout<<"\n \n Before Sorting based Mac48Address and store in staAllocated1 value of temp=="<<temp2;
// int count1=0;
// staAllocated1.clear();
// while(temp2){
//   auto startItt = m_staInfo.begin();
//       for(int i=0;i<noOfSTA;i++){
//         if(finalStaPairIndex[i].second==count1){
//           staAllocated1.push_back(*startItt);
//           count1++;

//         }
//         startItt++;
//         //break;
//       }
//       temp2--;

// }

std::cout<<"\n\n before staAllocated1 check how many m_staInfo there"<<m_staInfo.size();
int temp3=0;
staAllocated1.clear();
while(temp2){
  auto startItt = m_staInfo.begin();
 int dd = finalStaPairIndex[temp3].second;
      for(int i=0;i<dd;i++){
        // if(finalStaPairIndex[i].second==count1){
        //   staAllocated1.push_back(*startItt);
        //   count1++;

        //}
        startItt++;
        //break;
      }
      staAllocated1.push_back(*startItt);
      temp3++;

      temp2--;

}


auto checking=staAllocated1.begin();

  while(checking!=staAllocated1.end()){
          std::cout<<"\n staAllocated1-- "<< checking->first;
    checking++;
  }

std::cout<<"\n \n After Sorting based Mac48Address and store in staAllocated1"<<staAllocated1.size();



// selectiveMCS.clear();

// len=finalRUAlloc.size();
// ;
//   }
// }

  
}











////////////////////////////////////////////////COMPUTE DL OFDMA/////////////////////////////////////////////////////////
OfdmaManager::DlOfdmaInfo
RrOfdmaManager::ComputeDlOfdmaInfo (void)
{
  NS_LOG_FUNCTION (this);

  if (m_staInfo.empty ())
    {
      return DlOfdmaInfo ();
    }

  uint16_t bw = m_low->GetPhy ()->GetChannelWidth ();
//   uint16_t bw = m_bw;   // for TESTING only

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_staInfo.size ();
std::cout<<"m_sta_info size: ComputeDlOfdmaInfo "<<m_staInfo.size ()<<"\n";
  





HeRu::RuType ruType ;

if(v_dataStaPair.size()>1){
  //call your function
  
Largest_Weighted_First();
  std::cout<<"\n Largest_Weighted_First return\n";

  nRusAssigned=finalRUAlloc.size();
  std::cout<<"\nmin ru alloc size assigned to nru assigned"<<nRusAssigned<<"\n";

  if(nRusAssigned==0) //in case no feasible ru allocation for current input exists.
  {
    std::cout<<"if(nRusAssigned==0) \n";
    //std::size_t nRusAs = m_staInfo.size ();
    nRusAssigned = m_staInfo.size ();
    ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);
  }
}else{
  std::cout<<"inside else";
  std::cout<<"if(dataStaPair.size()>1): false, in else case\n";
  //std::size_t nRusAs = m_staInfo.size ();
  nRusAssigned = m_staInfo.size ();
  ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);
}

//HeRu::RuType ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);

///////////////////////////////////////////////////////

//std::cout<<nRusAssigned << " stations are being assigned a " << ruType << " RU\n";
 // NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");

std::cout<<"\n before Intilaizing DlOfdmaInfo dlOfdmaInfo\n";
  DlOfdmaInfo dlOfdmaInfo;
  auto staInfoIt = m_staInfo.begin (); // iterator over the list of candidate receivers

  if(staAllocated1.empty()){
    for (std::size_t i = 0; i < nRusAssigned; i++){
      
      std::cout<<"\n staAllocated1 is empty"<<i;
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      dlOfdmaInfo.staInfo.insert (*staInfoIt);
      staInfoIt++;
    }
  }else if(staAllocated1.size()>0){
        auto sti = staAllocated1.begin (); // iterator over the list of candidate receivers
        for (std::size_t i = 0; i < finalRUAlloc.size(); i++){
           std::cout<<"\n staAllocated1 is > 0"<<i;
        	NS_ASSERT (sti != staAllocated1.end ());
          dlOfdmaInfo.staInfo.insert (*sti);
          sti++;
        }
    }
  
    std::cout<<"\n After DlOfdmaInfo initialization\n";

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // if not all the stations are assigned an RU, the first station to serve next
  // time is the first one that was not served this time
  if (nRusAssigned < m_staInfo.size ())
    {
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      m_startStation = staInfoIt->second.aid;
      std::cout<<"Next station to serve has AID=" << m_startStation<<"\n";
    }

  auto firstSTA = m_staInfo.begin (); 
  m_startStation=firstSTA->second.aid; ////// Thinkkkkkk
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  NS_LOG_DEBUG ("Next station to serve has AID=" << m_startStation);


    std::cout<<"\n before InitTxVectorAndParams func call\n";



  InitTxVectorAndParams (dlOfdmaInfo.staInfo, ruType, m_dlMuAckSequence);
  dlOfdmaInfo.params = m_txParams;

    std::cout<<"\n after InitTxVectorAndParams func call\n";


 

  if (ruType == HeRu::RU_2x996_TONE)
    {
      HeRu::RuSpec ru = {true, ruType, 1};
      NS_LOG_DEBUG ("STA " << m_staInfo.front ().first << " assigned " << ru);
      m_txVector.SetRu (ru, m_staInfo.front ().second.aid);
    }
  else
    {
      std::vector<bool> primary80MHzSet {true};

      if (bw == 160)
        {
          primary80MHzSet.push_back (false);
          bw = 80;
        }

      auto mapIt = dlOfdmaInfo.staInfo.begin ();
        if(mappedRuAllocated.size()!=0){
        std::size_t ru26 = 0;
        std::size_t ru52 = 0;
        std::size_t ru106 = 0;
        std::size_t ru242 = 0;
        std::size_t ru484 = 0;
          std::vector<int>::size_type len = mappedRuAllocated.size();
          std::cout<<"\n mappedRuAllocated size="<<len;
          for (unsigned i=0; i<len; i++){
           HeRu::RuSpec ru;
        switch(mappedRuAllocated[i]){
          case HeRu::RU_26_TONE: ru26++;
                     ru = {true, mappedRuAllocated[i],ru26};
                   break;
          case HeRu::RU_52_TONE: ru52++;
                     ru = {true, mappedRuAllocated[i],ru52};
                   break;
          case HeRu::RU_106_TONE: ru106++;
                     ru = {true, mappedRuAllocated[i],ru106};
                   break;
          case HeRu::RU_242_TONE: ru242++;
                    ru = {true, mappedRuAllocated[i],ru242};
                   break;
          case HeRu::RU_484_TONE: ru484++;
                    ru = {true, mappedRuAllocated[i],ru484};
                   break;
        default: break;
          }
            std::cout<<"\n STA ----" << mapIt->first << " assigned---- " << ru;
           NS_LOG_DEBUG ("STA " << mapIt->first << " assigned " << ru);
           m_txVector.SetRu (ru, mapIt->second.aid);
           mapIt++;
          }
        }
        else{

              for (auto primary80MHz : primary80MHzSet)
                {
                  for (std::size_t ruIndex = 1; ruIndex <= HeRu::m_heRuSubcarrierGroups.at ({bw, ruType}).size (); ruIndex++)
                    {
                      NS_ASSERT (mapIt != dlOfdmaInfo.staInfo.end ());
                      HeRu::RuSpec ru = {primary80MHz, ruType, ruIndex};
                      NS_LOG_DEBUG ("STA " << mapIt->first << " assigned " << ru);
                      m_txVector.SetRu (ru, mapIt->second.aid);
                      mapIt++;
                    }
                }
             }
}
  dlOfdmaInfo.txVector = m_txVector;

  if (m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR
      || m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
    {
      Ptr<WifiRemoteStationManager> stationManager = GetWifiRemoteStationManager ();
      // The Trigger Frame to be returned is built from the TX vector used for the DL MU PPDU
      // (i.e., responses will use the same set of RUs) and modified to ensure that responses
      // are sent at a rate not higher than MCS 5.
      dlOfdmaInfo.trigger = GetTriggerFrameHeader (dlOfdmaInfo.txVector, 5);
      dlOfdmaInfo.trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (dlOfdmaInfo.trigger, m_txParams));
      SetTargetRssi (dlOfdmaInfo.trigger);
    }

finalRUAlloc.clear();
mappedRuAllocated.clear();


  std::cout<<"\nsetting mcs before\n";

if(random_MCS.size()>1){
  std::cout<<"\nsetting mcs inside loop\n";
unsigned int x=0;
auto userInfoMap = dlOfdmaInfo.txVector.GetHeMuUserInfoMap ();
for (auto& userInfo : userInfoMap)
    {
      uint8_t mcs = random_MCS.at(x);
      NS_LOG_FUNCTION("MCS"<<mcs);
 dlOfdmaInfo.txVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
                                                     WifiPhy::GetHeMcs (mcs),
                                                     userInfo.second.nss});
x++;
    }
  random_MCS.clear();
 }
  

  std::cout<<"\nsetting mcs after\n";
staAllocated1.clear();





// auto userInfoMap = dlMuTxVector.GetHeMuUserInfoMap ();

//   for (auto& userInfo : userInfoMap)
//     {
//       uint8_t mcs = std::min (userInfo.second.mcs.GetMcsValue (), maxMcs);
//       dlMuTxVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
//                                                      WifiPhy::GetHeMcs (mcs),
//                                                      userInfo.second.nss});
//     }


  return dlOfdmaInfo;
}

CtrlTriggerHeader
RrOfdmaManager::GetTriggerFrameHeader (WifiTxVector dlMuTxVector, uint8_t maxMcs)
{
  auto userInfoMap = dlMuTxVector.GetHeMuUserInfoMap ();

  for (auto& userInfo : userInfoMap)
    {
      uint8_t mcs = std::min (userInfo.second.mcs.GetMcsValue (), maxMcs);
      dlMuTxVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
                                                     WifiPhy::GetHeMcs (mcs),
                                                     userInfo.second.nss});
    }

  return CtrlTriggerHeader (TriggerFrameType::MU_BAR_TRIGGER, dlMuTxVector);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




OfdmaManager::UlOfdmaInfo
RrOfdmaManager::ComputeUlOfdmaInfo (void)
{
  CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);
  trigger.SetUlLength (m_txVector.GetLength ());
  SetTargetRssi (trigger);

  UlOfdmaInfo ulOfdmaInfo;
  ulOfdmaInfo.params = m_txParams;
  ulOfdmaInfo.trigger = trigger;

  return ulOfdmaInfo;
}

} //namespace ns3
