/*
 * canal_degradado_v2.cc
 *
 * TFG: Analisis de la conectividad y latencia en redes de UAV mediante
 * el protocolo MAVLink en entornos simulados.
 *
 * Canal de comunicaciones degradable entre dos namespaces de Linux.
 *
 * TOPOLOGIA DE ESTA VERSION (patron del ejemplo oficial tap-wifi-dumbbell)
 * -----------------------------------------------------------------------
 *
 *   ns-dronA                                              ns-dronB
 *   192.168.30.10                                    192.168.31.10
 *       |                                                    |
 *   [tap-left]                                        [tap-right]
 *       |                                                    |
 *  +----------+   CSMA        +------+   P2P      +------+   CSMA   +----------+
 *  | fantasmaA|--(~0 retardo)-|router|--(retardo--|router|--(~0)----| fantasmaB|
 *  |   .2     |               | izq  |  y perdida)| der  |          |   .2     |
 *  +----------+               |  .1  |            |  .1  |          +----------+
 *                             +------+            +------+
 *                          10.100.0.1          10.100.0.2
 *
 * PointToPointChannel no hace deteccion de portadora: solo hay dos
 * extremos, luego no puede haber colision. Cada paquete se programa de
 * forma independiente para llegar 'retardo' despues de salir, asi que
 * pueden viajar muchos simultaneamente. La capacidad en vuelo es el
 * producto tasa x retardo: 10 Mbit/s x 0,1 s = 1 Mbit = ~1600 paquetes,
 * frente a 1 solo en CSMA.
 *
 * Los segmentos CSMA se mantienen pegados a las taps porque TapBridge
 * necesita un dispositivo con semantica Ethernet para manejar ARP y las
 * tramas de descubrimiento; PointToPointNetDevice no las soporta.
 *
 * NOTA: los mensajes usan std::cout y no NS_LOG_UNCOND porque el perfil
 * de compilacion 'optimized' elimina los macros de log.
 */

#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tap-bridge-module.h"

#include <iostream>

using namespace ns3;

int
main(int argc, char* argv[])
{
    // Parametros configurables desde linea de comandos
    double latenciaMs = 0.0;          // retardo por trayecto (ida)
    double perdida = 0.0;             // probabilidad de descarte [0,1]
    double duracionSeg = 600.0;       // duracion de la simulacion
    bool perdidaBidireccional = false; // aplicar perdida en ambos sentidos
    std::string tasaEnlace = "10Mbps"; // tasa del enlace intermedio
    std::string tapIzq = "tap-left";
    std::string tapDer = "tap-right";

    // Modo tiempo real. Debe fijarse ANTES de instanciar el simulador.
    // BestEffort evita que la simulacion aborte si se queda retrasada
    // respecto al reloj de pared (critico en maquina virtual).
    Config::SetDefault("ns3::RealtimeSimulatorImpl::SynchronizationMode",
                       StringValue("BestEffort"));
    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));
    // Checksums reales: imprescindible, los paquetes salen a un SO real.
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

    CommandLine cmd(__FILE__);
    cmd.AddValue("latencia", "Retardo de propagacion por trayecto, en ms", latenciaMs);
    cmd.AddValue("perdida", "Probabilidad de descarte de paquete [0,1]", perdida);
    cmd.AddValue("duracion", "Duracion de la simulacion en segundos", duracionSeg);
    cmd.AddValue("bidireccional", "Aplicar la perdida en ambos sentidos", perdidaBidireccional);
    cmd.AddValue("tasa", "Tasa del enlace intermedio, p.ej. 10Mbps", tasaEnlace);
    cmd.AddValue("tapIzq", "Nombre de la interfaz TAP izquierda", tapIzq);
    cmd.AddValue("tapDer", "Nombre de la interfaz TAP derecha", tapDer);
    cmd.Parse(argc, argv);

    // Nodos
    //   fantasmaA / fantasmaB: no simulan nada, solo sostienen el
    //   TapBridge que conecta con el namespace real de Linux.
    //   routerIzq / routerDer: encaminan entre el segmento CSMA local
    //   y el enlace punto a punto intermedio.
    Ptr<Node> fantasmaA = CreateObject<Node>();
    Ptr<Node> routerIzq = CreateObject<Node>();
    Ptr<Node> routerDer = CreateObject<Node>();
    Ptr<Node> fantasmaB = CreateObject<Node>();

    NodeContainer todos;
    todos.Add(fantasmaA);
    todos.Add(routerIzq);
    todos.Add(routerDer);
    todos.Add(fantasmaB);

    // El router va primero en cada contenedor para que reciba la .1,
    // que es la direccion que usaran los namespaces como pasarela.
    NodeContainer nodosIzq;
    nodosIzq.Add(routerIzq);
    nodosIzq.Add(fantasmaA);

    NodeContainer nodosDer;
    nodosDer.Add(routerDer);
    nodosDer.Add(fantasmaB);

    NodeContainer nodosP2P;
    nodosP2P.Add(routerIzq);
    nodosP2P.Add(routerDer);

    // Segmentos CSMA de acceso: rapidos y sin retardo apreciable.
    // Su unica funcion es dar semantica Ethernet a las TAP.
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    csma.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", StringValue("1000p"));

    NetDeviceContainer devIzq = csma.Install(nodosIzq);
    NetDeviceContainer devDer = csma.Install(nodosDer);

    // Enlace intermedio: aqui viven el retardo y la perdida.
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue(tasaEnlace));
    p2p.SetChannelAttribute("Delay", TimeValue(Seconds(latenciaMs / 1000.0)));
    p2p.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", StringValue("1000p"));

    NetDeviceContainer devP2P = p2p.Install(nodosP2P);

    // Modelo de perdida sobre el enlace intermedio.
    // Por defecto solo en el sentido izquierda -> derecha, para
    // mantener la coherencia con la version 1. Con --bidireccional=1
    // se aplica tambien en el sentido de vuelta, que es lo habitual
    // en un enlace de radio degradado.
    if (perdida > 0.0)
    {
        Ptr<RateErrorModel> emIda = CreateObject<RateErrorModel>();
        emIda->SetAttribute("ErrorRate", DoubleValue(perdida));
        emIda->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        devP2P.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(emIda));

        if (perdidaBidireccional)
        {
            Ptr<RateErrorModel> emVuelta = CreateObject<RateErrorModel>();
            emVuelta->SetAttribute("ErrorRate", DoubleValue(perdida));
            emVuelta->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
            devP2P.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(emVuelta));
        }
    }

    // Pila IP y direccionamiento
    //   192.168.30.0/24  segmento izquierdo  (router .1, fantasma .2)
    //   10.100.0.0/30    enlace intermedio   (.1 y .2)
    //   192.168.31.0/24  segmento derecho    (router .1, fantasma .2)
    // Los namespaces usan la .10 de su segmento, fuera del rango que
    // asigna ns-3, para que no haya colision de direcciones.
    InternetStackHelper internet;
    internet.Install(todos);

    Ipv4AddressHelper dir;

    dir.SetBase("192.168.30.0", "255.255.255.0");
    dir.Assign(devIzq);

    dir.SetBase("10.100.0.0", "255.255.255.252");
    dir.Assign(devP2P);

    dir.SetBase("192.168.31.0", "255.255.255.0");
    dir.Assign(devDer);

    // Puentes con el mundo real. Modo UseLocal: la TAP la configura
    // el sistema operativo (preparar_namespace.sh) y TapBridge suplanta
    // su direccion MAC.
    TapBridgeHelper puente;
    puente.SetAttribute("Mode", StringValue("UseLocal"));

    puente.SetAttribute("DeviceName", StringValue(tapIzq));
    puente.Install(fantasmaA, devIzq.Get(1));

    puente.SetAttribute("DeviceName", StringValue(tapDer));
    puente.Install(fantasmaB, devDer.Get(1));

    // Encaminamiento: sin esto los routers no saben llegar al otro lado.
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Informe de arranque
    double rttTeorico = 2.0 * latenciaMs;

    std::cout << "=== Canal degradado TFG v2 ===" << std::endl;
    std::cout << "Latencia (ida): " << latenciaMs << " ms" << std::endl;
    std::cout << "RTT teorico:    " << rttTeorico << " ms" << std::endl;
    std::cout << "Perdida:        " << (perdida * 100.0) << " %"
              << (perdidaBidireccional ? " (ambos sentidos)" : " (solo ida)")
              << std::endl;
    std::cout << "Tasa enlace:    " << tasaEnlace << std::endl;
    std::cout << "Duracion:       " << duracionSeg << " s" << std::endl;
    std::cout << "TAP izq:        " << tapIzq << "  -> pasarela 192.168.30.1" << std::endl;
    std::cout << "TAP der:        " << tapDer << " -> pasarela 192.168.31.1" << std::endl;
    std::cout << "==============================" << std::endl;
    std::cout << "Interfaces TAP creadas. Simulacion en marcha." << std::endl;
    std::cout << "Ejecuta ahora preparar_namespace.sh en otra terminal." << std::endl;

    Simulator::Stop(Seconds(duracionSeg));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "Simulacion finalizada." << std::endl;
    return 0;
}
