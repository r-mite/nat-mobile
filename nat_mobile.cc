/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/tcp-header.h"

#include "ns3/point-to-point-layout-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"

#include <sstream>
#include <math.h>

#include "ns3/stats-module.h"

#define NET_MASKC "255.255.255.0"
#define NET_MASKB "255.255.0.0"
#define NET_ADDR1 "192.168.0.0"
#define NET_ADDR2 "172.16.0.0"
#define NET_ADDR3 "172.16.1.0"
#define FIRST_NO "0.0.1.1"

#define SIM_START 00.0
#define SIM_STOP 40.0
#define DATA_MBYTES 500
#define PORT 50000
#define PROG_DIR "prog/"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("exp04-TcpFlowMonitor");

// packet size generated at the AP
static const uint32_t packetSize = 1420;


class NodeStatistics
{
public:
	NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas);

	void CheckStatistics(double time);

	void PhyCallback(std::string path, Ptr<const Packet> packet);
	void RxCallback(std::string path, Ptr<const Packet> packet, const Address &from);
	void PowerCallback(std::string path, uint8_t power, Mac48Address dest);
	void RateCallback(std::string path, uint32_t rate, Mac48Address dest);
	void SetPosition(Ptr<Node> node, Vector position);
	void AdvancePosition(Ptr<Node> node, int stepsSize, int stepsTime);
	Vector GetPosition(Ptr<Node> node);

	Gnuplot2dDataset GetDatafile();
	Gnuplot2dDataset GetPowerDatafile();

private:
	typedef std::vector<std::pair<Time, WifiMode> > TxTime;
	void SetupPhy(Ptr<WifiPhy> phy);
	Time GetCalcTxTime(WifiMode mode);

	std::map<Mac48Address, uint32_t> actualPower;
	std::map<Mac48Address, WifiMode> actualMode;
	uint32_t m_bytesTotal;
	double totalEnergy;
	double totalTime;
	Ptr<WifiPhy> myPhy;
	TxTime timeTable;
	Gnuplot2dDataset m_output;
	Gnuplot2dDataset m_output_power;
};

//コンストラクタ
NodeStatistics::NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas)
{
	Ptr<NetDevice> device = aps.Get(0);
	Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
	Ptr<WifiPhy> phy = wifiDevice->GetPhy();
	myPhy = phy;
	SetupPhy(phy);
	for (uint32_t j = 0; j < stas.GetN(); j++)
	{
		Ptr<NetDevice> staDevice = stas.Get(j);
		Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice>(staDevice);
		Mac48Address addr = wifiStaDevice->GetMac()->GetAddress();
		actualPower[addr] = 17;
		actualMode[addr] = phy->GetMode(0);
	}
	actualMode[Mac48Address("ff:ff:ff:ff:ff:ff")] = phy->GetMode(0);
	totalEnergy = 0;
	totalTime = 0;
	m_bytesTotal = 0;
	m_output.SetTitle("Throughput Mbits/s");
	m_output_power.SetTitle("Average Transmit Power");
}

//wifi設定
void
NodeStatistics::SetupPhy(Ptr<WifiPhy> phy)
{
	uint32_t nModes = phy->GetNModes();
	for (uint32_t i = 0; i < nModes; i++)
	{
		WifiMode mode = phy->GetMode(i);
		WifiTxVector txVector;
		txVector.SetMode(mode);
		timeTable.push_back(std::make_pair(phy->CalculateTxDuration(packetSize, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency(), 0, 0), mode));
	}
}

//タイマー
Time
NodeStatistics::GetCalcTxTime(WifiMode mode)
{
	for (TxTime::const_iterator i = timeTable.begin(); i != timeTable.end(); i++)
	{
		if (mode == i->second)
		{
			return i->first;
		}
	}
	NS_ASSERT(false);
	return Seconds(0);
}

//wifiのエネルギーと時間計算
void
NodeStatistics::PhyCallback(std::string path, Ptr<const Packet> packet)
{
	WifiMacHeader head;
	packet->PeekHeader(head);
	Mac48Address dest = head.GetAddr1();

	totalEnergy += actualPower[dest] * GetCalcTxTime(actualMode[dest]).GetSeconds();
	totalTime += GetCalcTxTime(actualMode[dest]).GetSeconds();

}

//下に同様の関数あり
void
NodeStatistics::PowerCallback(std::string path, uint8_t power, Mac48Address dest)
{
	double   txPowerBaseDbm = myPhy->GetTxPowerStart();
	double   txPowerEndDbm = myPhy->GetTxPowerEnd();
	uint32_t nTxPower = myPhy->GetNTxPower();
	double dbm;
	if (nTxPower > 1)
	{
		dbm = txPowerBaseDbm + power * (txPowerEndDbm - txPowerBaseDbm) / (nTxPower - 1);
	}
	else
	{
		NS_ASSERT_MSG(txPowerBaseDbm == txPowerEndDbm, "cannot have TxPowerEnd != TxPowerStart with TxPowerLevels == 1");
		dbm = txPowerBaseDbm;
	}
	actualPower[dest] = dbm;
}

//上記同様
void
NodeStatistics::RateCallback(std::string path, uint32_t rate, Mac48Address dest)
{
	actualMode[dest] = myPhy->GetMode(rate);
}

//パケットのサイズ
void
NodeStatistics::RxCallback(std::string path, Ptr<const Packet> packet, const Address &from)
{
	m_bytesTotal += packet->GetSize();
}

void
NodeStatistics::CheckStatistics(double time)
{

}

//移動端末の位置設定
void
NodeStatistics::SetPosition(Ptr<Node> node, Vector position)
{
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	mobility->SetPosition(position);
}

//移動端末の位置取得
Vector
NodeStatistics::GetPosition(Ptr<Node> node)
{
	Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
	return mobility->GetPosition();
}

//移動量計算ルーチン
void
NodeStatistics::AdvancePosition(Ptr<Node> node, int stepsSize, int stepsTime)
{
	Vector pos = GetPosition(node);
	double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime));
	m_bytesTotal = 0;
	double atm = pow(10, ((totalEnergy / stepsTime) / 10));
	totalEnergy = 0;
	totalTime = 0;
	m_output_power.Add(pos.x, atm);
	m_output.Add(pos.x, mbs);
	//pos.x += stepsSize;
	SetPosition(node, pos);
	NS_LOG_INFO("At time " << Simulator::Now().GetSeconds() << " sec; setting new position to " << pos);
	Simulator::Schedule(Seconds(stepsTime), &NodeStatistics::AdvancePosition, this, node, stepsSize, stepsTime);
}

Gnuplot2dDataset
NodeStatistics::GetDatafile()
{
	return m_output;
}

Gnuplot2dDataset
NodeStatistics::GetPowerDatafile()
{
	return m_output_power;
}

void PowerCallback(std::string path, uint8_t power, Mac48Address dest)
{
	NS_LOG_INFO((Simulator::Now()).GetSeconds() << " " << dest << " Power " << (int)power);
}

void RateCallback(std::string path, uint32_t rate, Mac48Address dest)
{
	NS_LOG_INFO((Simulator::Now()).GetSeconds() << " " << dest << " Rate " << rate);
}


int
main (int argc, char *argv[])
{
	//コマンドラインから
	CommandLine cmd;
	cmd.Parse(argc, argv);
	
	//wifiの強度なのかな
	double maxPower = 17;
	double minPower = 0;
	uint32_t powerLevels = 18;

	//何だろう
	uint32_t rtsThreshold = 2346;
	std::string manager = "ns3::ParfWifiManager";
	std::string outputFileName = "parf";
	/*
	int ap1_x = 0;
	int ap1_y = 0;
	int sta1_x = 5;
	int sta1_y = 0;
	uint32_t steps = 30;
	*/
	uint32_t stepsSize = 5;
	uint32_t stepsTime = 1;

	//uint32_t nLeftLeaf = 3;
	//uint32_t nRightLeaf = 3;
	//wifi側の個数
	uint32_t nWifi = 1;
	//lan側の個数
	uint32_t nLan = 1;
	//uint32_t nRouter = 4;

	//送信されるパケットの設定
	Config::SetDefault("ns3::OnOffApplication::PacketSize", UintegerValue(1024));
	Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("500kb/s"));
	Config::SetDefault("ns3::DropTailQueue::MaxPackets", UintegerValue(100));

	//中央ネットワークの両端
	NodeContainer p2pNodes;
	p2pNodes.Create(2);

	//中央ネットワークの設定
	PointToPointHelper p2pRouter;
	p2pRouter.SetDeviceAttribute("DataRate", StringValue("1Mbps"));
	p2pRouter.SetChannelAttribute("Delay", StringValue("5ms"));
	//p2pRouter.SetQueue("ns3::DropTailQueue", "MaxPackets", UintegerValue(10));

	//中央のネットワークデバイス
	NetDeviceContainer p2pDevice;
	p2pDevice = p2pRouter.Install(p2pNodes);


	NodeContainer p2pNodea;
	p2pNodea.Add(p2pNodes.Get(1));
	p2pNodea.Create(1);
	NetDeviceContainer p2pDevicea;
	p2pDevicea = p2pRouter.Install(p2pNodea);

	
	//lan側
	NodeContainer lanNodes;
	lanNodes.Add(p2pNodes.Get(1));
	lanNodes.Create(1);
	NetDeviceContainer lanDevices;
	lanDevices = p2pRouter.Install(lanNodes);
	
	
	NodeContainer p2pNodeb;
	p2pNodeb.Add(lanNodes.Get(1));
	p2pNodeb.Add(p2pNodea.Get(1));
	NetDeviceContainer p2pDeviceb;
	p2pDeviceb = p2pRouter.Install(p2pNodeb);
	
	NodeContainer p2pNodec;
	p2pNodec.Add(lanNodes.Get(1));
	p2pNodec.Create(1);
	NetDeviceContainer p2pDevicec;
	p2pDevicec = p2pRouter.Install(p2pNodec);
	
	NodeContainer p2pNoded;
	p2pNoded.Add(p2pNodea.Get(1));
	p2pNoded.Create(1);
	NetDeviceContainer p2pDeviced;
	p2pDeviced = p2pRouter.Install(p2pNoded);


	//wifi側
	NodeContainer wifiStaNodes;
	wifiStaNodes.Create(nWifi);
	NodeContainer wifiApNode = p2pNodes.Get(0);

	//NodeContainer wifiSta2;
	//wifiSta2.Create(nWifi);
	NodeContainer wifiAp2 = p2pNoded.Get(1);


	//wifi各種設定
	YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
	YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
	phy.SetChannel(channel.Create());

	WifiHelper wifi = WifiHelper::Default();
	wifi.SetRemoteStationManager("ns3::AarfWifiManager");

	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

	wifiPhy.SetChannel(wifiChannel.Create());

	NetDeviceContainer apDevices;
	NetDeviceContainer staDevices;
	NetDeviceContainer wifiDevices;

	NetDeviceContainer apDevice2;
	//NetDeviceContainer staDevice2;
	NetDeviceContainer wifiDevice2;
	

	//子機側の設定
	wifi.SetRemoteStationManager("ns3::MinstrelWifiManager", "RtsCtsThreshold", UintegerValue(rtsThreshold));
	wifiPhy.Set("TxPowerStart", DoubleValue(maxPower));
	wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));

	Ssid ssid = Ssid("ns-3-ssid");
	wifiMac.SetType("ns3::StaWifiMac",
		"Ssid", SsidValue(ssid),
		"ActiveProbing", BooleanValue(false));
	staDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiStaNodes.Get(0)));
	//staDevice2.Add(wifi.Install(wifiPhy, wifiMac, wifiSta2.Get(0)));

	//親機側の設定
	wifi.SetRemoteStationManager(manager, "DefaultTxPowerLevel", UintegerValue(maxPower), "RtsCtsThreshold", UintegerValue(rtsThreshold));
	wifiPhy.Set("TxPowerStart", DoubleValue(minPower));
	wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));
	wifiPhy.Set("TxPowerLevels", UintegerValue(powerLevels));

	wifiMac.SetType("ns3::ApWifiMac",
		"Ssid", SsidValue(ssid));
	apDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNode.Get(0)));
	apDevice2.Add(wifi.Install(wifiPhy, wifiMac, wifiAp2.Get(0)));

	wifiDevices.Add(staDevices);
	wifiDevices.Add(apDevices);

	wifiDevice2.Add(staDevice);
	wifiDevice2.Add(apDevice2);


	//モバイル設定
	MobilityHelper mobility;
	mobility.SetPositionAllocator("ns3::GridPositionAllocator",
		"MinX", DoubleValue(0.0),
		"MinY", DoubleValue(0.0),
		"DeltaX", DoubleValue(5.0),
		"DeltaY", DoubleValue(10.0),
		"GridWidth", UintegerValue(3),
		"LayoutType", StringValue("RowFirst"));
	
	mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)));
	//mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(wifiStaNodes);
	//mobility.Install(wifiSta2);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(wifiApNode);
	mobility.SetPositionAllocator("ns3::GridPositionAllocator",
		"MinX", DoubleValue(40.0),
		"MinY", DoubleValue(0.0),
		"DeltaX", DoubleValue(5.0),
		"DeltaY", DoubleValue(10.0),
		"GridWidth", UintegerValue(3),
		"LayoutType", StringValue("RowFirst"));
	
	mobility.Install(wifiAp2);

	//スタック設定
	InternetStackHelper stack;
	stack.Install(lanNodes);
	stack.Install(wifiApNode);
	stack.Install(wifiStaNodes);
	stack.Install(wifiAp2);
	//stack.Install(wifiSta2);

	//アドレス設定
	Ipv4AddressHelper address;

	address.SetBase(NET_ADDR2, NET_MASKC);
	Ipv4InterfaceContainer p2pInterfaces;
	p2pInterfaces = address.Assign(p2pDevice);

	address.SetBase(NET_ADDR3, NET_MASKC);
	Ipv4InterfaceContainer lanInterfaces;
	lanInterfaces = address.Assign(lanDevices);

	address.SetBase(NET_ADDR1, NET_MASKC);
	address.Assign(staDevices);
	address.Assign(apDevices);

	//address.Assign(staDevice2);
	address.Assign(apDevice2);
	address.Assign(wifiDevice2);

	//インターフェースをつけないといけないっぽい
	Ipv4InterfaceContainer i = address.Assign(wifiDevices);
	
	//UDPエコーサーバクライアント
	UdpEchoServerHelper echoServer(9);
	ApplicationContainer serverApps = echoServer.Install(p2pNodes.Get(1));
	serverApps.Start(Seconds(1.0));
	serverApps.Stop(Seconds(10.0));

	UdpEchoClientHelper echoClient(lanInterfaces.GetAddress(nLan), 9);
	echoClient.SetAttribute("MaxPackets", UintegerValue(1));
	echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
	echoClient.SetAttribute("PacketSize", UintegerValue(1024));

	ApplicationContainer clientApps = echoClient.Install(wifiStaNodes.Get(nWifi - 1));
	clientApps.Start(Seconds(2.0));
	clientApps.Stop(Seconds(10.0));
	
	Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	//シミュレータ内移動ルーチン
	NodeStatistics statistics = NodeStatistics(apDevices, staDevices);
	Simulator::Schedule(Seconds(0.5 + stepsTime), &NodeStatistics::AdvancePosition, &statistics, wifiStaNodes.Get(0), stepsSize, stepsTime);

	//シミュレーション
	Simulator::Stop(Seconds(SIM_STOP));
	Simulator::Run();
	Simulator::Destroy();
	return 0;



	/*
	//両側ネットワークの設定
	PointToPointHelper p2pLeaf;
	p2pLeaf.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
	p2pLeaf.SetChannelAttribute("Delay", StringValue("1ms"));
	p2pLeaf.SetQueue("ns3::DropTailQueue");

	//ダンベルネットワーク
	PointToPointDumbbellHelper net (nLeftLeaf, p2pLeaf, nRightLeaf, p2pLeaf, p2pRouter);

	//スタックを装着
	InternetStackHelper stack;
	net.InstallStack(stack);

	net.AssignIpv4Addresses(
		Ipv4AddressHelper(NET_ADDR1, NET_MASK1), 
		Ipv4AddressHelper(NET_ADDR2, NET_MASK2), 
		Ipv4AddressHelper(NET_ADDR3, NET_MASK1)
		);

	//onoff属性を設定
	OnOffHelper tcp("ns3::TcpSocketFactory", Address());
	tcp.SetAttribute("OnTime", StringValue("ns3::UniformRandomVariable[Min=3.0,max=5.00]"));
	tcp.SetAttribute("OffTime", StringValue("ns3::UniformRandomVariable[Min=0.1,max=0.50]"));
	tcp.SetAttribute("DataRate", StringValue("450Kbps"));
	tcp.SetAttribute("PacketSize", UintegerValue(1024));

	//TCPソースアプリケーション
	ApplicationContainer source;
	for(uint32_t i=0; i<net.LeftCount(); i++){
		AddressValue remoteAddress(InetSocketAddress(net.GetRightIpv4Address(i), PORT));
		tcp.SetAttribute("Remote", remoteAddress);
		source.Add(tcp.Install(net.GetLeft(i)));
	}
	source.Start(Seconds(SIM_START));
	source.Stop(Seconds(SIM_STOP));

	//TCPシンクアプリケーション
	ApplicationContainer sink;
	for(uint32_t i=0; i<net.RightCount(); i++){
		Address sinkAddress(InetSocketAddress(net.GetRightIpv4Address(i), PORT));
		PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkAddress);
		sinkHelper.SetAttribute("Local", AddressValue(sinkAddress));
		sink.Add(sinkHelper.Install(net.GetRight(i)));
	}
	sink.Start(Seconds(SIM_START));
	sink.Stop(Seconds(SIM_STOP));

	//アニメーション設定
	net.BoundingBox(1, 1, 100, 100);

	//ルーティング設定
	Ipv4GlobalRoutingHelper::PopulateRoutingTables();
	*/
	/*
	//フローモニター設定
	FlowMonitorHelper flowmon;
	flowmon.SetMonitorAttribute("JitterBinWidth", ns3::DoubleValue(0.001));
	flowmon.SetMonitorAttribute("DelayBinWidth", ns3::DoubleValue(0.001));
	flowmon.SetMonitorAttribute("PacketSizeBinWidth", ns3::DoubleValue(20));
	Ptr<FlowMonitor> monitor = flowmon.InstallAll();
	*/
	//Simulator::Stop(Seconds(SIM_STOP));
	//Simulator::Run();
	/*
	//Run後に記述
	monitor->CheckForLostPackets();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
	std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

	std::cout<< "----------\n";
	for(std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i=stats.begin(); i!=stats.end(); ++i){
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
		if((t.sourceAddress=="172.16.1.1" && t.destinationAddress=="172.18.1.1") ||
			(t.sourceAddress=="172.16.2.1" && t.destinationAddress=="172.18.2.1") ||
			(t.sourceAddress=="172.16.3.1" && t.destinationAddress=="172.18.3.1")
			){
				std::cout << "Flow "
					<< i->first
					<< " ("
					<< t.sourceAddress
					<< " -> "
					<< t.destinationAddress
					<< ")\n";
				std::cout << "   Delay sum: "
					<< i->second.delaySum
					<< "\n";
				std::cout << "   Jitter sum: "
					<< i->second.jitterSum
					<< "\n";
				std::cout << "   Tx Bytes: "
					<< i->second.txBytes
					<< "\n";
				std::cout << "   Rx Bytes: "
					<< i->second.rxBytes
					<< "\n";
				std::cout << "   Tx Packets: "
					<< i->second.txPackets
					<< "\n";
				std::cout << "   Rx Packets: "
					<< i->second.rxPackets
					<< "\n";
				std::cout << "   Lost Packets: "
					<< i->second.lostPackets
					<< "\n";
				std::cout << "   Times Forwarded: "
					<< i->second.timesForwarded
					<< "\n\n";
				std::cout << " Troughput: "
					<< i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024
					<< " Kbps\n";
				std::cout << "----------\n";
		}
	}

	//トレースファイル生成
	AsciiTraceHelper ascii;
	std::string fname = std::string(PROG_DIR) + "exp04-TCP-flowmon.xml";
	Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(fname);
	monitor->SerializeToXmlFile(fname, true, true);
	*/
	//Simulator::Destroy();
	//return 0;

}
