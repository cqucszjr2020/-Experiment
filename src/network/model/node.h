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
 *
 *          Yuping Wu<pingw@cqu.edu.cn>
 */
#ifndef NODE_H
#define NODE_H

#include <vector>

#include "ns3/object.h"
#include "ns3/callback.h"
#include "ns3/ptr.h"
#include "ns3/net-device.h"

#include "ns3/vector.h"

#include "ns3/mobility-model.h"
#include<map>

namespace ns3 {

class Application;
class Packet;
class Address;
class Time;

/*
 * Pingw Struct
 */
//RV存放node发送的ctrtag，若node在危险区域则存储其数据到RV
struct RVInform{
	uint32_t NodeId_ping; //节点id
//	uint32_t Dir;  //运动方向
	Vector Vec_ping; //车辆速度3D  double
	Vector Pos_ping;  //车辆位置3D double
//	double Acce; //加速度
	uint32_t Hv;//健康值
	RVInform(uint32_t idp, Vector vecp,Vector positionp,uint32_t hv) {
		NodeId_ping=idp;
		Vec_ping=vecp;
		Pos_ping=positionp;
		Hv = hv;
	}
	bool operator ==(const uint32_t &s) {
		return NodeId_ping == s;
	}
};
//RVNV存放RV发送的ctrtag
struct RVNVInform{
	uint32_t RV_Id; //节点id
//	uint32_t Dir;  //运动方向
	Vector RV_Vec; //车辆速度3D  double
	Vector RV_Pos;  //车辆位置3D double
//	double Acce; //加速度
	uint32_t hV;//健康值
	RVNVInform(uint32_t id_NV, Vector vec_NV,Vector position_NV,uint32_t hv) {
		RV_Id=id_NV;
		RV_Vec=vec_NV;
		RV_Pos=position_NV;
		hV = hv;
	}
	bool operator ==(const uint32_t &s) {
		return RV_Id == s;
	}
};
//NV普通节点存储其前车信息
struct NVfrontcarInfo{
	uint32_t FCId;//前车ID
	Vector FC_Pos;//前车位置
	Vector FC_Vec;//前车速度
	double min_dis;//与前车的最小距离
	NVfrontcarInfo(uint32_t fc_id, Vector vec_fc,Vector position_fc,uint32_t minDis){
		FCId = fc_id;
		FC_Pos = position_fc;
		FC_Vec = vec_fc;
		min_dis = minDis;
	}
	bool operator ==(const uint32_t &s) {
			return FCId == s;
		}
};
//Rsu存放收到的RV的Ctr消息
struct CtrRsuRvMessage {
	uint32_t R_Id; //节点id
//	uint32_t R_Dir;  //运动方向
	Vector R_Vec; //车辆速度3D  double
	Vector R_Position;  //车辆位置3D double
//	double R_Acce; //加速度
	uint32_t r_hv;
	CtrRsuRvMessage(uint32_t rid, Vector rvec, Vector rpos,uint32_t hv) {
		R_Id=rid;
		R_Vec=rvec;
		R_Position=rpos;
		r_hv = hv;
	}
	bool operator ==(const uint32_t &s) {
		return R_Id == s;
	}
};
//Rsu存放收到的NV的Ctr消息
struct CtrRsuNvMessage {
	uint32_t NodeId; //节点id
//	uint32_t Dir;  //运动方向
	Vector Vec; //车辆速度3D  double
	Vector Position;  //车辆位置3D double
//	double Acce; //加速度
	uint32_t n_hv;//健康值
	CtrRsuNvMessage(uint32_t id, Vector vec, Vector pos,uint32_t hv) {
		NodeId=id;
		Vec=vec;
		Position=pos;
		n_hv = hv;
	}
	bool operator ==(const uint32_t &s) {
		return NodeId == s;
	}
};



/**
 * \ingroup network
 *
 * \brief A network Node.
 *
 * This class holds together:
 *   - a list of NetDevice objects which represent the network interfaces
 *     of this node which are connected to other Node instances through
 *     Channel instances.
 *   - a list of Application objects which represent the userspace
 *     traffic generation applications which interact with the Node
 *     through the Socket API.
 *   - a node Id: a unique per-node identifier.
 *   - a system Id: a unique Id used for parallel simulations.
 *
 * Every Node created is added to the NodeList automatically.
 */
class Node : public Object
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  Node();
  /**
   * \param systemId a unique integer used for parallel simulations.
   */
  Node(uint32_t systemId);

  virtual ~Node();

  /**
   * \returns the unique id of this node.
   * 
   * This unique id happens to be also the index of the Node into
   * the NodeList. 
   */
  uint32_t GetId (void) const;

  /**
   * In the future, ns3 nodes may have clock that returned a local time
   * different from the virtual time Simulator::Now().
   * This function is currently a placeholder to ease the development of this feature.
   * For now, it is only an alias to Simulator::Now()
   *
   * \return The time as seen by this node
   */
  Time GetLocalTime (void) const;

  /**
   * \returns the system id for parallel simulations associated
   *          to this node.
   */
  uint32_t GetSystemId (void) const;

  /**
   * \brief Associate a NetDevice to this node.
   *
   * \param device NetDevice to associate to this node.
   * \returns the index of the NetDevice into the Node's list of
   *          NetDevice.
   */
  uint32_t AddDevice (Ptr<NetDevice> device);
  /**
   * \brief Retrieve the index-th NetDevice associated to this node.
   *
   * \param index the index of the requested NetDevice
   * \returns the requested NetDevice.
   */
  Ptr<NetDevice> GetDevice (uint32_t index) const;
  /**
   * \returns the number of NetDevice instances associated
   *          to this Node.
   */
  uint32_t GetNDevices (void) const;

  /**
   * \brief Associate an Application to this Node.
   *
   * \param application Application to associate to this node.
   * \returns the index of the Application within the Node's list
   *          of Application.
   */
  uint32_t AddApplication (Ptr<Application> application);
  /**
   * \brief Retrieve the index-th Application associated to this node.
   *
   * \param index the index of the requested Application
   * \returns the requested Application.
   */
  Ptr<Application> GetApplication (uint32_t index) const;

  /**
   * \returns the number of Application instances associated to this Node.
   */
  uint32_t GetNApplications (void) const;

  /**
   * A protocol handler
   *
   * \param device a pointer to the net device which received the packet
   * \param packet the packet received
   * \param protocol the 16 bit protocol number associated with this packet.
   *        This protocol number is expected to be the same protocol number
   *        given to the Send method by the user on the sender side.
   * \param sender the address of the sender
   * \param receiver the address of the receiver; Note: this value is
   *                 only valid for promiscuous mode protocol
   *                 handlers.  Note:  If the L2 protocol does not use L2
   *                 addresses, the address reported here is the value of 
   *                 device->GetAddress().
   * \param packetType type of packet received
   *                   (broadcast/multicast/unicast/otherhost); Note:
   *                   this value is only valid for promiscuous mode
   *                   protocol handlers.
   */
  typedef Callback<void,Ptr<NetDevice>, Ptr<const Packet>,uint16_t,const Address &,
                   const Address &, NetDevice::PacketType> ProtocolHandler;
  /**
   * \param handler the handler to register
   * \param protocolType the type of protocol this handler is 
   *        interested in. This protocol type is a so-called
   *        EtherType, as registered here:
   *        http://standards.ieee.org/regauth/ethertype/eth.txt
   *        the value zero is interpreted as matching all
   *        protocols.
   * \param device the device attached to this handler. If the
   *        value is zero, the handler is attached to all
   *        devices on this node.
   * \param promiscuous whether to register a promiscuous mode handler
   */
  void RegisterProtocolHandler (ProtocolHandler handler, 
                                uint16_t protocolType,
                                Ptr<NetDevice> device,
                                bool promiscuous=false);
  /**
   * \param handler the handler to unregister
   *
   * After this call returns, the input handler will never
   * be invoked anymore.
   */
  void UnregisterProtocolHandler (ProtocolHandler handler);

  /**
   * A callback invoked whenever a device is added to a node.
   */
  typedef Callback<void,Ptr<NetDevice> > DeviceAdditionListener;
  /**
   * \param listener the listener to add
   *
   * Add a new listener to the list of listeners for the device-added
   * event. When a new listener is added, it is notified of the existence
   * of all already-added devices to make discovery of devices easier.
   */
  void RegisterDeviceAdditionListener (DeviceAdditionListener listener);
  /**
   * \param listener the listener to remove
   *
   * Remove an existing listener from the list of listeners for the 
   * device-added event.
   */
  void UnregisterDeviceAdditionListener (DeviceAdditionListener listener);


  //Ping function
  void SetAcceleration(double acce);
  void SetVelocity(double vec);
  void SetPosition(Vector position);
  double GetAcceleration(void);
  double GetVelocity(void);
  Vector GetPosition(void);

  void SetRsuFlag(bool bflag);   //设置车辆是否为RSU
  bool GetRsuFlag(void) const;
  void SetRVFlag(bool bflag);   //设置车辆是否为危险车辆 pingw
  bool GetRVFlag(void) const;
  bool GetClusterMemberFlag(void) const;
  void SetClusterMemberFlag(bool sflag);   //设置车辆是否入簇

  void SetHV(uint32_t hv);//设置节点健康值
	uint32_t GetHV() const;//获取节点健康值
	void SetBT(double t); // 设置该节点bsm生成周期
	double GetBT() const;   //获取该节点bsm生成周期


	/*
	 * pingw对结构体操作
	 */
	void SetCtrRsuRvMessage(uint32_t rv,const CtrRsuRvMessage &Ctr); //设置rsu车辆保存的rv信息
	std::map<uint32_t,CtrRsuRvMessage> GetCtrRsuRvMessage(void);
	void RemoveCtrRsuRvM(void);
	void SetCtrRsuNvMessage(uint32_t nv,const CtrRsuNvMessage &Ctr); //设置rsu车辆保存的Nv信息
	std::map<uint32_t,CtrRsuNvMessage> GetCtrRsuNvMessage(void);
	void RemoveCtrRsuNvM(void);
	void SetRVInform(uint32_t NV_id,const RVInform &rvinform);//设置RV存储的该危险区域内node的信息
	std::map<uint32_t,RVInform>GetRVInform(void);
	void RemoveRVInform(void);
	void SetRVNVInform(uint32_t RV_id,const RVNVInform &rvnvinform);//设置RVNV存储RV信息
	std::map<uint32_t,RVNVInform>GetRVNVInform(void);
	void RemoveRVNVInform(void);
	void SetNVfrontcarInfo(const NVfrontcarInfo &frontcarInfo);//设置普通节点存储的前车fc信息
	std::vector<NVfrontcarInfo> GetNVfrontcarInfo(void);
	void RemoveNVfrontcarInfo(void);



  /**
   * \returns true if checksums are enabled, false otherwise.
   */
  static bool ChecksumEnabled (void);


protected:
  /**
   * The dispose method. Subclasses must override this method
   * and must chain up to it by calling Node::DoDispose at the
   * end of their own DoDispose method.
   */
  virtual void DoDispose (void);
  virtual void DoInitialize (void);
private:

  /**
   * \brief Notifies all the DeviceAdditionListener about the new device added.
   * \param device the added device to notify.
   */
  void NotifyDeviceAdded (Ptr<NetDevice> device);

  /**
   * \brief Receive a packet from a device in non-promiscuous mode.
   * \param device the device
   * \param packet the packet
   * \param protocol the protocol
   * \param from the sender
   * \returns true if the packet has been delivered to a protocol handler.
   */
  bool NonPromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address &from);
  /**
   * \brief Receive a packet from a device in promiscuous mode.
   * \param device the device
   * \param packet the packet
   * \param protocol the protocol
   * \param from the sender
   * \param to the destination
   * \param packetType the packet type
   * \returns true if the packet has been delivered to a protocol handler.
   */
  bool PromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                 const Address &from, const Address &to, NetDevice::PacketType packetType);
  /**
   * \brief Receive a packet from a device.
   * \param device the device
   * \param packet the packet
   * \param protocol the protocol
   * \param from the sender
   * \param to the destination
   * \param packetType the packet type
   * \param promisc true if received in promiscuous mode
   * \returns true if the packet has been delivered to a protocol handler.
   */
  bool ReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet>, uint16_t protocol,
                          const Address &from, const Address &to, NetDevice::PacketType packetType, bool promisc);

  /**
   * \brief Finish node's construction by setting the correct node ID.
   */
  void Construct (void);

  /**
   * \brief Protocol handler entry.
   * This structure is used to demultiplex all the protocols.
   */
  struct ProtocolHandlerEntry {
    ProtocolHandler handler; //!< the protocol handler
    Ptr<NetDevice> device;   //!< the NetDevice
    uint16_t protocol;       //!< the protocol number
    bool promiscuous;        //!< true if it is a promiscuous handler
  };

  /// Typedef for protocol handlers container
  typedef std::vector<struct Node::ProtocolHandlerEntry> ProtocolHandlerList;
  /// Typedef for NetDevice addition listeners container
  typedef std::vector<DeviceAdditionListener> DeviceAdditionListenerList;

  uint32_t    m_id;         //!< Node id for this node
  uint32_t    m_sid;        //!< System id for this node
  std::vector<Ptr<NetDevice> > m_devices; //!< Devices associated to this node
  std::vector<Ptr<Application> > m_applications; //!< Applications associated to this node
  ProtocolHandlerList m_handlers; //!< Protocol handlers in the node
  DeviceAdditionListenerList m_deviceAdditionListeners; //!< Device addition listeners in the node

  /*
   * ping 变量
   */
  double accel_value;        //车辆加速度
  double vec; //车辆速度
  Vector position;  //车辆位置
  bool rsu_flag;  //是否为RSU
  bool Rv_flag;//是否为RV危险车辆 pingw
  double MAX_ITT;//传输间隔
  uint32_t HV;//健康值
  bool clustermember_flag;  //未入簇的车辆节点
  /*
   * pingw结构体
   */
  std::map<uint32_t,CtrRsuRvMessage>ctrRvInform;//RSU存放RV发送的ctr包
  std::map<uint32_t,CtrRsuNvMessage>ctrNvInfom; //rsu存放nv发送的ctr包

  std::vector<NVfrontcarInfo>NVfrontcarinfo;//普通node节点存储前车信息

  std::map<uint32_t,RVInform>RVinform;//RV存放该危险区域内node的ID、vec
  std::map<uint32_t,RVNVInform>RVNVinform;//RVNV存放附近区域内RV的信息

};

} // namespace ns3

#endif /* NODE_H */
