/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006 Georgia Tech Research Corporation, INRIA
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
 * Authors: George F. Riley<riley@ece.gatech.edu>
 *          Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
 
#include "node.h"
#include "node-list.h"
#include "net-device.h"
#include "application.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/object-vector.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/global-value.h"
#include "ns3/boolean.h"
#include<map>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Node");

NS_OBJECT_ENSURE_REGISTERED (Node);

/**
 * \brief A global switch to enable all checksums for all protocols.
 */
static GlobalValue g_checksumEnabled  = GlobalValue ("ChecksumEnabled",
                                                     "A global switch to enable all checksums for all protocols",
                                                     BooleanValue (false),
                                                     MakeBooleanChecker ());

TypeId 
Node::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Node")
    .SetParent<Object> ()
    .SetGroupName("Network")
    .AddConstructor<Node> ()
    .AddAttribute ("DeviceList", "The list of devices associated to this Node.",
                   ObjectVectorValue (),
                   MakeObjectVectorAccessor (&Node::m_devices),
                   MakeObjectVectorChecker<NetDevice> ())
    .AddAttribute ("ApplicationList", "The list of applications associated to this Node.",
                   ObjectVectorValue (),
                   MakeObjectVectorAccessor (&Node::m_applications),
                   MakeObjectVectorChecker<Application> ())
    .AddAttribute ("Id", "The id (unique integer) of this Node.",
                   TypeId::ATTR_GET, // allow only getting it.
                   UintegerValue (0),
                   MakeUintegerAccessor (&Node::m_id),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("SystemId", "The systemId of this node: a unique integer used for parallel simulations.",
                   TypeId::ATTR_GET | TypeId::ATTR_SET,
                   UintegerValue (0),
                   MakeUintegerAccessor (&Node::m_sid),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}

Node::Node()
  : m_id (0),
    m_sid (0)
{
  NS_LOG_FUNCTION (this);
  Construct ();
}

Node::Node(uint32_t sid)
  : m_id (0),
    m_sid (sid)
{ 
  NS_LOG_FUNCTION (this << sid);
  Construct ();
}

void
Node::Construct (void)
{
  NS_LOG_FUNCTION (this);
  m_id = NodeList::Add (this);
}

Node::~Node ()
{
  NS_LOG_FUNCTION (this);
}

uint32_t
Node::GetId (void) const
{
  NS_LOG_FUNCTION (this);
  return m_id;
}

Time
Node::GetLocalTime (void) const
{
  NS_LOG_FUNCTION (this);
  return Simulator::Now ();
}

uint32_t
Node::GetSystemId (void) const
{
  NS_LOG_FUNCTION (this);
  return m_sid;
}

uint32_t
Node::AddDevice (Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << device);
  uint32_t index = m_devices.size ();
  m_devices.push_back (device);
  device->SetNode (this);
  device->SetIfIndex (index);
  device->SetReceiveCallback (MakeCallback (&Node::NonPromiscReceiveFromDevice, this));
  Simulator::ScheduleWithContext (GetId (), Seconds (0.0), 
                                  &NetDevice::Initialize, device);
  NotifyDeviceAdded (device);
  return index;
}
Ptr<NetDevice>
Node::GetDevice (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  NS_ASSERT_MSG (index < m_devices.size (), "Device index " << index <<
                 " is out of range (only have " << m_devices.size () << " devices).");
  return m_devices[index];
}
uint32_t 
Node::GetNDevices (void) const
{
  NS_LOG_FUNCTION (this);
  return m_devices.size ();
}

uint32_t 
Node::AddApplication (Ptr<Application> application)
{
  NS_LOG_FUNCTION (this << application);
  uint32_t index = m_applications.size ();
  m_applications.push_back (application);
  application->SetNode (this);
  Simulator::ScheduleWithContext (GetId (), Seconds (0.0), 
                                  &Application::Initialize, application);
  return index;
}
Ptr<Application> 
Node::GetApplication (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  NS_ASSERT_MSG (index < m_applications.size (), "Application index " << index <<
                 " is out of range (only have " << m_applications.size () << " applications).");
  return m_applications[index];
}
uint32_t 
Node::GetNApplications (void) const
{
  NS_LOG_FUNCTION (this);
  return m_applications.size ();
}

void 
Node::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_deviceAdditionListeners.clear ();
  m_handlers.clear ();
  for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
       i != m_devices.end (); i++)
    {
      Ptr<NetDevice> device = *i;
      device->Dispose ();
      *i = 0;
    }
  m_devices.clear ();
  for (std::vector<Ptr<Application> >::iterator i = m_applications.begin ();
       i != m_applications.end (); i++)
    {
      Ptr<Application> application = *i;
      application->Dispose ();
      *i = 0;
    }
  m_applications.clear ();
  Object::DoDispose ();
}
void 
Node::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
       i != m_devices.end (); i++)
    {
      Ptr<NetDevice> device = *i;
      device->Initialize ();
    }
  for (std::vector<Ptr<Application> >::iterator i = m_applications.begin ();
       i != m_applications.end (); i++)
    {
      Ptr<Application> application = *i;
      application->Initialize ();
    }

  Object::DoInitialize ();
}

void
Node::RegisterProtocolHandler (ProtocolHandler handler, 
                               uint16_t protocolType,
                               Ptr<NetDevice> device,
                               bool promiscuous)
{
  NS_LOG_FUNCTION (this << &handler << protocolType << device << promiscuous);
  struct Node::ProtocolHandlerEntry entry;
  entry.handler = handler;
  entry.protocol = protocolType;
  entry.device = device;
  entry.promiscuous = promiscuous;

  // On demand enable promiscuous mode in netdevices
  if (promiscuous)
    {
      if (device == 0)
        {
          for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
               i != m_devices.end (); i++)
            {
              Ptr<NetDevice> dev = *i;
              dev->SetPromiscReceiveCallback (MakeCallback (&Node::PromiscReceiveFromDevice, this));
            }
        }
      else
        {
          device->SetPromiscReceiveCallback (MakeCallback (&Node::PromiscReceiveFromDevice, this));
        }
    }

  m_handlers.push_back (entry);
}

void
Node::UnregisterProtocolHandler (ProtocolHandler handler)
{
  NS_LOG_FUNCTION (this << &handler);
  for (ProtocolHandlerList::iterator i = m_handlers.begin ();
       i != m_handlers.end (); i++)
    {
      if (i->handler.IsEqual (handler))
        {
          m_handlers.erase (i);
          break;
        }
    }
}

bool
Node::ChecksumEnabled (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  BooleanValue val;
  g_checksumEnabled.GetValue (val);
  return val.Get ();
}

bool
Node::PromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                const Address &from, const Address &to, NetDevice::PacketType packetType)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from << &to << packetType);
  return ReceiveFromDevice (device, packet, protocol, from, to, packetType, true);
}

bool
Node::NonPromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                   const Address &from)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from);
  return ReceiveFromDevice (device, packet, protocol, from, device->GetAddress (), NetDevice::PacketType (0), false);
}

bool
Node::ReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                         const Address &from, const Address &to, NetDevice::PacketType packetType, bool promiscuous)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from << &to << packetType << promiscuous);
  NS_ASSERT_MSG (Simulator::GetContext () == GetId (), "Received packet with erroneous context ; " <<
                 "make sure the channels in use are correctly updating events context " <<
                 "when transferring events from one node to another.");
  NS_LOG_DEBUG ("Node " << GetId () << " ReceiveFromDevice:  dev "
                        << device->GetIfIndex () << " (type=" << device->GetInstanceTypeId ().GetName ()
                        << ") Packet UID " << packet->GetUid ());
  bool found = false;

  for (ProtocolHandlerList::iterator i = m_handlers.begin ();
       i != m_handlers.end (); i++)
    {
      if (i->device == 0 ||
          (i->device != 0 && i->device == device))
        {
          if (i->protocol == 0 || 
              i->protocol == protocol)
            {
              if (promiscuous == i->promiscuous)
                {
                  i->handler (device, packet, protocol, from, to, packetType);
                  found = true;
                }
            }
        }
    }
  return found;
}
void 
Node::RegisterDeviceAdditionListener (DeviceAdditionListener listener)
{
  NS_LOG_FUNCTION (this << &listener);
  m_deviceAdditionListeners.push_back (listener);
  // and, then, notify the new listener about all existing devices.
  for (std::vector<Ptr<NetDevice> >::const_iterator i = m_devices.begin ();
       i != m_devices.end (); ++i)
    {
      listener (*i);
    }
}
void 
Node::UnregisterDeviceAdditionListener (DeviceAdditionListener listener)
{
  NS_LOG_FUNCTION (this << &listener);
  for (DeviceAdditionListenerList::iterator i = m_deviceAdditionListeners.begin ();
       i != m_deviceAdditionListeners.end (); i++)
    {
      if ((*i).IsEqual (listener))
        {
          m_deviceAdditionListeners.erase (i);
          break;
         }
    }
}
 
void 
Node::NotifyDeviceAdded (Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << device);
  for (DeviceAdditionListenerList::iterator i = m_deviceAdditionListeners.begin ();
       i != m_deviceAdditionListeners.end (); i++)
    {
      (*i) (device);
    }  
}
 
//Ping function
void Node::SetAcceleration(double acce) {
	accel_value = acce;
}
void Node::SetVelocity(double rvec) {
	vec = rvec;
}
void Node::SetPosition(Vector rposition) {
	position = rposition;
}
double Node::GetAcceleration(void) {
	return accel_value;
}
double Node::GetVelocity(void) {
	return vec;
}
Vector Node::GetPosition(void) {
	return position;
}
void Node::SetRsuFlag(bool rflag) {
	rsu_flag = rflag;
}
bool Node::GetRsuFlag(void) const {
	return rsu_flag;
}
void Node::SetRVFlag(bool rvflag) {
	Rv_flag = rvflag;
}
bool Node::GetRVFlag(void) const {
	return Rv_flag;
}

bool Node::GetClusterMemberFlag(void) const {
	return clustermember_flag;
}
void Node::SetClusterMemberFlag(bool sflag) //设置未入簇车辆是否发送过
		{
	clustermember_flag = sflag;
}
void Node::SetHV(uint32_t hv){
	//设置节点健康值
	HV = hv;
}
uint32_t Node::GetHV() const{
	//获取节点健康值
	return HV;
}
void Node::SetBT(double t) // 设置该节点是否收到bsm生成周期
{
	MAX_ITT = t;
}
double Node::GetBT() const{//获取该节点是否有bsm生成周期
	return MAX_ITT;
}

/*
* pingw对结构体操作
*/
void Node::SetCtrRsuRvMessage(uint32_t rvid,const CtrRsuRvMessage &Ctr) {
	ctrRvInform.insert(std::pair<uint32_t,CtrRsuRvMessage>(rvid,Ctr));
}
std::map<uint32_t,CtrRsuRvMessage> Node::GetCtrRsuRvMessage(void) {
	return ctrRvInform;
}
void Node::RemoveCtrRsuRvM(void){
	ctrRvInform.clear();
}
void Node::SetCtrRsuNvMessage(uint32_t nvid,const CtrRsuNvMessage &Ctr) {
	ctrNvInfom.insert(std::pair<uint32_t,CtrRsuNvMessage>(nvid,Ctr));
}
std::map<uint32_t,CtrRsuNvMessage> Node::GetCtrRsuNvMessage(void) {
	return ctrNvInfom;
}
void Node::RemoveCtrRsuNvM(void){
	ctrNvInfom.clear();
}
void Node::SetRVInform(uint32_t NV_id,const RVInform &rvinform){
//	RVinform[NV_id]=rvinform;
	RVinform.insert(std::pair<uint32_t,RVInform >(NV_id,rvinform));
//	RVinform.insert(std::map<uint32_t,RVInform>::value_type(NV_id,rvinform));
}
std::map<uint32_t,RVInform> Node::GetRVInform(void){
	return RVinform;
}
void Node::RemoveRVInform(void){
	RVinform.erase(RVinform.begin(), RVinform.end());
}
void Node::SetRVNVInform(uint32_t RV_id,const RVNVInform &rvnvinform){
//	RVNVinform[RV_id]=rvnvinform;
	RVNVinform.insert(std::pair<uint32_t,RVNVInform>(RV_id,rvnvinform));
//	RVNVinform.insert(std::map<uint32_t,RVNVInform>::value_type(RV_id,rvnvinform));
}
std::map<uint32_t,RVNVInform>Node::GetRVNVInform(void){
	return RVNVinform;
}
void Node::RemoveRVNVInform(void){
	RVNVinform.erase(RVNVinform.begin(), RVNVinform.end());
}
void Node::SetNVfrontcarInfo(const NVfrontcarInfo &frontcarInfo){
	NVfrontcarinfo.push_back(frontcarInfo);
}
std::vector<NVfrontcarInfo> Node::GetNVfrontcarInfo(void){
	return NVfrontcarinfo;
}
void Node::RemoveNVfrontcarInfo(void){
	NVfrontcarinfo.clear();
}


} // namespace ns3
