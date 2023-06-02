#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/opengym-module.h"
//#include "ns3/csma-module.h"
#include "ns3/propagation-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/header.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-l3-protocol.h"

#include <fstream>
#include <string>
#include <math.h>
#include <ctime>   //timestampi
#include <iomanip> // put_time
#include <vector>
#include <deque>
#include <algorithm>
#include <csignal>
#include "scenario.h"

using namespace std;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OpenGym");

void installTrafficGenerator(Ptr<ns3::Node> fromNode, Ptr<ns3::Node> toNode, int port, string offeredLoad, double startTime);
void PopulateARPcache();
void recordHistory();

double envStepTime = 0.1;
double simulationTime = 10; //seconds
double current_time = 0.0;
bool verbose = false;
int end_delay = 0;
bool dry_run = false;

Ptr<FlowMonitor> monitor;
FlowMonitorHelper flowmon;
ofstream outfile ("scratch/RLinWiFi-Decentralized-v01/CW_data.csv", fstream::out);

uint32_t CW = 0;
int nWifi = 2;

// Our data structure for the scenario needs to be a vector of [history_length, 2]
uint32_t history_length = 20;

//adicionado por sheila
uint32_t n_line = 2; 

//deque<float> history;
std::vector<std::deque<float>> history(10); //funcionando adicionado por sheila

string type = "discrete";
bool non_zero_start = false;
Scenario *wifiScenario;


/*
Define observation space
*/
Ptr<OpenGymSpace> MyGetObservationSpace(void)
{
    float low = 0.0;
    float high = 10.0;
    std::vector<uint32_t> shape = {
        history_length,
    };
    std::string dtype = TypeNameGet<float>();
    Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    if (verbose)
        NS_LOG_UNCOND("MyGetObservationSpace: " << space);
    return space;
}

/*
Define action space
*/
Ptr<OpenGymSpace> MyGetActionSpace(void)
{
    float low = 0.0;
    float high = 10.0;
    std::vector<uint32_t> shape = {
        1,
    };
    std::string dtype = TypeNameGet<float>();
    Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
    if (verbose)
        NS_LOG_UNCOND("MyGetActionSpace: " << space);
    return space;
}

/*
Define extra info. Optional
*/
//uint64_t g_rxPktNum = 0;
//uint64_t g_txPktNum = 0;

std::vector<uint64_t> g_rxPktNum = {
        2,
};
std::vector<uint64_t> g_txPktNum = {
        2,
};

double jain_index(void)
{
    double flowThr;
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    double nominator;
    double denominator;
    double n=0;
    double station_id = 0;
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {
        flowThr = i->second.rxBytes;
        flowThr /= wifiScenario->getStationUptime(station_id, current_time);
        if(flowThr>0){
            nominator += flowThr;
            denominator += flowThr*flowThr;
            n++;
        }
        station_id++;
    }
    nominator *= nominator;
    denominator *= n;
    return nominator/denominator;
}

std::string MyGetExtraInfo(void)
{
    static float ticks = 0.0;
    static float lastValue = 0.0;
    std::string myInfo;    
    
    for (int sta_id = 1; sta_id <= nWifi; sta_id++)
    {
         float obs = g_rxPktNum[sta_id] - lastValue;
         lastValue = g_rxPktNum[sta_id];
         ticks += envStepTime;

         float sentMbytes = obs * (1500 - 20 - 8 - 8) * 8.0 / 1024 / 1024;

          myInfo = std::to_string(sentMbytes);
          myInfo = myInfo + "|" + to_string(CW) + "|";
          myInfo = myInfo + to_string(wifiScenario->getActiveStationCount(ticks)) + "|";
          myInfo = myInfo + to_string(jain_index());
    }
    if (verbose)
        NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
    return myInfo;
}

/*
Execute received actions
*/
bool MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
    if (verbose)
        NS_LOG_UNCOND("MyExecuteActions: " << action);

    Ptr<OpenGymBoxContainer<float>> box = DynamicCast<OpenGymBoxContainer<float>>(action);
    std::vector<float> actionVector = box->GetData();

    if (type == "discrete")
    {
        CW = pow(2, 4+actionVector.at(0));
    }
    else if (type == "continuous")
    {
        CW = pow(2, actionVector.at(0) + 4);
    }
    else if (type == "direct_continuous")
    {
        CW = actionVector.at(0);
    }
    else
    {
        std::cout << "Unsupported agent type!" << endl;
        exit(0);
    }

    uint32_t min_cw = 16;
    uint32_t max_cw = 1024;

    CW = min(max_cw, max(CW, min_cw));
    outfile << current_time << "," << CW << endl;

    if(!dry_run){
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(CW));
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(CW));
    }
    return true;
}

float MyGetReward(void)
{
    static float ticks = 0.0;
    static uint32_t last_packets = 0;
    static float last_reward = 0.0;
    ticks += envStepTime;

    for (int sta_id = 1; sta_id <= nWifi; sta_id++)
    {    
        float res = g_rxPktNum[sta_id] - last_packets;
        float reward = res * (1500 - 20 - 8 - 8) * 8.0 / 1024 / 1024 / (5 * 150 * envStepTime) * 10;

        last_packets = g_rxPktNum[sta_id];

        if (ticks <= 2 * envStepTime)
           return 0.0;

        if (verbose)
           NS_LOG_UNCOND("MyGetReward: " << reward);

        if(reward>1.0f || reward<0.0f)
           reward = last_reward;
        last_reward = reward;
    }
    return last_reward;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer> MyGetObservation()
{
    recordHistory();

    // Antes um vetor de 300 posicoes:
    /*std::vector<uint32_t> shape = {
        history_length,
    };*/
    //Agora uma matriz [2x300] com 2 linha e cada linha com 300 colunas:
    std::vector<uint32_t> shape = {
        n_line, history_length,
    };
    
    Ptr<OpenGymBoxContainer<float>> box = CreateObject<OpenGymBoxContainer<float>>(shape);
    for (uint32_t k = 0; k < history.size(); k++) //para percorrer/acessar as Stations
    {
        for (uint32_t i = 0; i < history.size(); i++) //para acessar o conteudo da Station
        {
            if (history[k][i] >= -100 && history[k][i] <= 100)
               box->AddValue(history[k][i]);
            else
               box->AddValue(0);
        }
    }
    for (uint32_t i = history.size(); i < history_length; i++)
    {
        box->AddValue(0);
    }
    if (verbose)
        NS_LOG_UNCOND("MyGetObservation: " << box);
    return box;
}

bool MyGetGameOver(void)
{
    // bool isGameOver = (ns3::Simulator::Now().GetSeconds() > simulationTime + end_delay + 1.0);
    return false;
}

void ScheduleNextStateRead(double envStepTime, Ptr<OpenGymInterface> openGymInterface)
{
    Simulator::Schedule(Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, openGymInterface);
    openGymInterface->NotifyCurrentState();
}

void recordHistory()
{
    // Keep track of the observations
    // We will define them as the error rate of the last `envStepTime` seconds
    //static uint32_t last_rx = 0;            // Previously received packets
    //static uint32_t last_tx = 0;            // Previously transmitted packets
    
    std::vector<uint32_t> last_rx = {
        2,
    };
    std::vector<uint32_t> last_tx = {
        2,
    };
    
    static uint32_t calls = 0;              // Number of calls to this function
    calls++;
    current_time += envStepTime;
    
    for (int sta_id = 1; sta_id <= nWifi; sta_id++)
    {

         float received = g_rxPktNum[sta_id] - last_rx[sta_id];  // Received packets since the last observation
         float sent = g_txPktNum[sta_id] - last_tx[sta_id];      // Sent (...)
         float errs = sent - received;           // Errors (...)
         float ratio;

         ratio = errs / sent;
         //history.push_front(ratio);
         history[0].push_back(ratio);
         history[1].push_back(ratio);

         // Remove the oldest observation if we have filled the history
         //if (history.size() > history_length)
         if (history[0].size() > history_length && history[1].size() > history_length)
         {
             //history.pop_back();
             history[0].pop_back();
             history[1].pop_back();
         }

         // Replace the last observation with the current one
         last_rx[sta_id] = g_rxPktNum[sta_id];
         last_tx[sta_id] = g_txPktNum[sta_id];

         if (calls < history_length && non_zero_start)
         {   
            // Schedule the next observation if we are not at the end of the simulation
            Simulator::Schedule(Seconds(envStepTime), &recordHistory);
         }
         else if (calls == history_length && non_zero_start)
         {
             g_rxPktNum[sta_id] = 0;
             g_txPktNum[sta_id] = 0;
             last_rx[sta_id] = 0;
             last_tx[sta_id] = 0;
         }
   }
}

/*void packetReceived(Ptr<const Packet> packet)
{
    NS_LOG_DEBUG("Client received a packet of " << packet->GetSize() << " bytes");
    g_rxPktNum++;
}*/

void packetReceivedWithAck(Ptr< const Packet > packet, Ptr< Ipv4 > ipv4, uint32_t interface)
{
    NS_LOG_DEBUG("Client received a packet of " << packet->GetSize() << " bytes");
    
    // Received IP Address with Ack
    Ipv4Header ipv4Header;
    packet->PeekHeader(ipv4Header);
    Ipv4Address RxsourceAddr = ipv4Header.GetSource ();
    std::cout <<" * packetReceived Source-Addr: " << RxsourceAddr << endl;
    uint32_t RxAddr = RxsourceAddr.Get();
    int Rxoctet4 = int (RxAddr & 0xFF); //Ip Address last octet
    std::cout <<" * packetReceived RxAddr int octeto4: " << Rxoctet4 << endl;
    std::cout << "-----------------------------" << "\n\n" << std::endl;
    
    g_rxPktNum[Rxoctet4]++;
    
    std::cout << ".............." << std::endl;
    std::cout << " ---- Vector Vec_g_rxPktNum counter: " << g_rxPktNum[1]<< std::endl;
    std::cout << " ---- Vector Vec_g_rxPktNum counter: " << g_rxPktNum[2]<< std::endl;
}

void packetSent(Ptr< const Packet > packet, Ptr< Ipv4 > ipv4, uint32_t interface)
{
    // Transmited IP Address
    Ipv4Header ipv4Header;
    packet->PeekHeader(ipv4Header);
    Ipv4Address TxsourceAddr = ipv4Header.GetSource ();
    std::cout <<" - packetSent Source-Addr: " << TxsourceAddr << endl;
    uint32_t TxAddr = TxsourceAddr.Get();
    int Txoctet4 = int (TxAddr & 0xFF); //Ip Address last octet
    std::cout <<" - packetSent TxAddr int octeto4: " << Txoctet4 << endl;
    std::cout << "--------------" << "\n" << std::endl;
    
    g_txPktNum[Txoctet4]++;
    
    std::cout << ".............." << std::endl;
    std::cout << " ---- Vector Vec_g_txPktNum counter: " << g_txPktNum[1]<< std::endl;
    std::cout << " ---- Vector Vec_g_txPktNum counter: " << g_txPktNum[2]<< std::endl;
}

void set_phy(int nWifi, int guardInterval, NodeContainer &wifiStaNode, NodeContainer &wifiApNode, YansWifiPhyHelper &phy)
{
    Ptr<MatrixPropagationLossModel> lossModel = CreateObject<MatrixPropagationLossModel>();
    lossModel->SetDefaultLoss(50);

    wifiStaNode.Create(nWifi);
    wifiApNode.Create(1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    Ptr<YansWifiChannel> chan = channel.Create();
    chan->SetPropagationLossModel(lossModel);
    chan->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

    phy = YansWifiPhyHelper::Default();
    phy.SetChannel(chan);

    // Set guard interval
    phy.Set("GuardInterval", TimeValue(NanoSeconds(guardInterval)));
}

void set_nodes(int channelWidth, int rng, int32_t simSeed, NodeContainer wifiStaNode, NodeContainer wifiApNode, YansWifiPhyHelper phy, WifiMacHelper mac, WifiHelper wifi, NetDeviceContainer &apDevice)
{
    // Set the access point details
    Ssid ssid = Ssid("ns3-80211ax");

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false),
                "BE_MaxAmpduSize", UintegerValue(0));

    NetDeviceContainer staDevice;
    staDevice = wifi.Install(phy, mac, wifiStaNode);

    mac.SetType("ns3::ApWifiMac",
                "EnableBeaconJitter", BooleanValue(false),
                "Ssid", SsidValue(ssid));

    apDevice = wifi.Install(phy, mac, wifiApNode);

    // Set channel width
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue(channelWidth));

    // mobility.
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    positionAlloc->Add(Vector(1.0, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.Install(wifiApNode);
    mobility.Install(wifiStaNode);
    /* Internet stack*/
    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNode);

    //Random
    if(simSeed!=-1)
        RngSeedManager::SetSeed(simSeed);
    RngSeedManager::SetRun(rng);

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staNodeInterface;
    Ipv4InterfaceContainer apNodeInterface;

    staNodeInterface = address.Assign(staDevice);
    apNodeInterface = address.Assign(apDevice);

    if (!dry_run)
    {
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(CW));
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(CW));
    }
    else
    {
        NS_LOG_UNCOND("Default CW");
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MinCw", UintegerValue(16));
        Config::Set("/$ns3::NodeListPriv/NodeList/*/$ns3::Node/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BE_Txop/$ns3::QosTxop/MaxCw", UintegerValue(1024));
    }
}

void set_sim(bool tracing, bool dry_run, int warmup, uint32_t openGymPort, YansWifiPhyHelper phy, NetDeviceContainer apDevice, int end_delay, Ptr<FlowMonitor> &monitor, FlowMonitorHelper &flowmon)
{
    monitor = flowmon.InstallAll();
    monitor->SetAttribute("StartTime", TimeValue(Seconds(warmup)));

    if (tracing)
    {
        phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
        phy.EnablePcap("cw", apDevice.Get(0));
    }

    Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface>(openGymPort);
    openGymInterface->SetGetActionSpaceCb(MakeCallback(&MyGetActionSpace));
    openGymInterface->SetGetObservationSpaceCb(MakeCallback(&MyGetObservationSpace));
    openGymInterface->SetGetGameOverCb(MakeCallback(&MyGetGameOver));
    openGymInterface->SetGetObservationCb(MakeCallback(&MyGetObservation));
    openGymInterface->SetGetRewardCb(MakeCallback(&MyGetReward));
    openGymInterface->SetGetExtraInfoCb(MakeCallback(&MyGetExtraInfo));
    openGymInterface->SetExecuteActionsCb(MakeCallback(&MyExecuteActions));

    // if (!dry_run)
    // {
    if (non_zero_start)
    {
        Simulator::Schedule(Seconds(1.0), &recordHistory);
        Simulator::Schedule(Seconds(envStepTime * history_length + 1.0), &ScheduleNextStateRead, envStepTime, openGymInterface);
    }
    else
        Simulator::Schedule(Seconds(1.0), &ScheduleNextStateRead, envStepTime, openGymInterface);
    // }

    Simulator::Stop(Seconds(simulationTime + end_delay + 1.0 + envStepTime*(history_length+1)));

    NS_LOG_UNCOND("Simulation started");
    Simulator::Run();
}

void signalHandler(int signum)
{
    cout << "Interrupt signal " << signum << " received.\n";
    exit(signum);
}

int main(int argc, char *argv[])
{
    //int nWifi = 2; //5;
    bool tracing = false;
    bool useRts = false;
    int mcs = 11;
    int channelWidth = 20;
    int guardInterval = 800;
    string offeredLoad = "150";
    int port = 1025;
    string outputCsv = "cw.csv";
    string scenario = "basic";
    dry_run = false;

    int rng = 1;
    int warmup = 1;

    uint32_t openGymPort = 5555;
    int32_t simSeed = -1;

    signal(SIGTERM, signalHandler);
    outfile << "SimulationTime,CW" << endl;

    CommandLine cmd;
    cmd.AddValue("openGymPort", "Specify port number. Default: 5555", openGymPort);
    cmd.AddValue("CW", "Value of Contention Window", CW);
    cmd.AddValue("historyLength", "Length of history window", history_length);
    cmd.AddValue("nWifi", "Number of wifi 802.11ax STA devices", nWifi);
    cmd.AddValue("verbose", "Tell echo applications to log if true", verbose);
    cmd.AddValue("tracing", "Enable pcap tracing", tracing);
    cmd.AddValue("rng", "Number of RngRun", rng);
    cmd.AddValue("simTime", "Simulation time in seconds. Default: 10s", simulationTime);
    cmd.AddValue("envStepTime", "Step time in seconds. Default: 0.1s", envStepTime);
    cmd.AddValue("agentType", "Type of agent actions: discrete, continuous", type);
    cmd.AddValue("nonZeroStart", "Start only after history buffer is filled", non_zero_start);
    cmd.AddValue("scenario", "Scenario for analysis: basic, convergence, reaction", scenario);
    cmd.AddValue("dryRun", "Execute scenario with BEB and no agent interaction", dry_run);
    cmd.AddValue("seed", "Random seed", simSeed);

    cmd.Parse(argc, argv);
    // history_length*=2;

    NS_LOG_UNCOND("Ns3Env parameters:");
    NS_LOG_UNCOND("--nWifi: " << nWifi);
    NS_LOG_UNCOND("--simulationTime: " << simulationTime);
    NS_LOG_UNCOND("--openGymPort: " << openGymPort);
    NS_LOG_UNCOND("--envStepTime: " << envStepTime);
    NS_LOG_UNCOND("--seed: " << simSeed);
    NS_LOG_UNCOND("--agentType: " << type);
    NS_LOG_UNCOND("--scenario: " << scenario);
    NS_LOG_UNCOND("--dryRun: " << dry_run);

    if (verbose)
    {
        LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

    if (useRts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("0"));
    }

    NodeContainer wifiStaNode;
    NodeContainer wifiApNode;
    YansWifiPhyHelper phy;
    set_phy(nWifi, guardInterval, wifiStaNode, wifiApNode, phy);

    WifiMacHelper mac;
    WifiHelper wifi;

    wifi.SetStandard(WIFI_PHY_STANDARD_80211ax_5GHZ);

    std::ostringstream oss;
    oss << "HeMcs" << mcs;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(oss.str()),
                                 "ControlMode", StringValue(oss.str()));

    //802.11ac PHY
    /*
  phy.Set ("ShortGuardEnabled", BooleanValue (0));
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  "DataMode", StringValue ("VhtMcs8"),
  "ControlMode", StringValue ("VhtMcs8"));
 */
    //802.11n PHY
    //phy.Set ("ShortGuardEnabled", BooleanValue (1));
    //wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
    //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
    //                              "DataMode", StringValue ("HtMcs7"),
    //                              "ControlMode", StringValue ("HtMcs7"));

    NetDeviceContainer apDevice;
    set_nodes(channelWidth, rng, simSeed, wifiStaNode, wifiApNode, phy, mac, wifi, apDevice);

    ScenarioFactory helper = ScenarioFactory(nWifi, wifiStaNode, wifiApNode, port, offeredLoad, history_length);
    wifiScenario = helper.getScenario(scenario);

    // if (!dry_run)
    // {
    if (non_zero_start)
        end_delay = envStepTime * history_length + 1.0;
    else
        end_delay = 0.0;
    // }

    wifiScenario->installScenario(simulationTime + end_delay + envStepTime, envStepTime, MakeCallback(&packetReceivedWithAck));

    // Config::ConnectWithoutContext("/NodeList/0/ApplicationList/*/$ns3::OnOffApplication/Tx", MakeCallback(&packetSent));
    //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback(&packetSent));
    
    Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/Tx", MakeCallback(&packetSent));
    
    Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/Rx", MakeCallback(&packetReceivedWithAck));

    wifiScenario->PopulateARPcache();
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();


    set_sim(tracing, dry_run, warmup, openGymPort, phy, apDevice, end_delay, monitor, flowmon);

    double flowThr;
    for (int sta_id = 1; sta_id <= nWifi; sta_id++)
    {
        float res =  g_rxPktNum[sta_id] * (1500 - 20 - 8 - 8) * 8.0 / 1024 / 1024;
        printf("Sent mbytes: %.2f\tThroughput: %.3f", res, res/simulationTime);
    }
    ofstream myfile;
    myfile.open(outputCsv, ios::app);

    /* Contents of CSV output file
    Timestamp, CW, nWifi, RngRun, SourceIP, DestinationIP, Throughput
    */
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
    {
        auto time = std::time(nullptr); //Get timestamp
        auto tm = *std::localtime(&time);
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        flowThr = i->second.rxBytes * 8.0 / simulationTime / 1000 / 1000;
        NS_LOG_UNCOND("Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\tThroughput: " << flowThr << " Mbps\tTime: " << i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds() << " s\tRx packets " << i->second.rxPackets);
        myfile << std::put_time(&tm, "%Y-%m-%d %H:%M") << "," << CW << "," << nWifi << "," << RngSeedManager::GetRun() << "," << t.sourceAddress << "," << t.destinationAddress << "," << flowThr;
        myfile << std::endl;
    }
    myfile.close();

    Simulator::Destroy();
    for (int sta_id = 1; sta_id <= nWifi; sta_id++)
    {
         NS_LOG_UNCOND("Packets registered by handler: " << g_rxPktNum[sta_id] << " Packets" << endl);
    }

    return 0;
}
