/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

using Navtech.IASDK.Events;
using Navtech.IASDK.Networking;
using System;
using System.Linq;
using System.Threading;

namespace Navtech.TestClient
{
    /// <summary>
    /// This console application is a very simple example of a radar client application.
    /// It connects to a radar when the application starts and then responds to the following messages:
    ///  - Configuration Message -> Displays some radar config params after initial connection. It also issues a start FFT Data message once the config is received
    ///  - FFT Data -> When a FFT Data message is received we display the radar rotation speed based on the frequency of messages arriving
    /// Data is only processed for 6 seconds before the application closes
    /// </summary>
    internal class Program
    {
        private static readonly RadarTcpClient RadarTcpClient = new RadarTcpClient();
        private static ushort _lastAzimuth;
        private static ushort _PacketCount;

        private static DateTime _lastRotationReset;

        private static void Main(string[] args)
        {
            Console.WriteLine("Test Client Starting");
            //Configure the event handlers for data messages and connection state change
            RadarTcpClient.OnConfigurationData += ConfigurationDataHandler;
            RadarTcpClient.OnFftData += FftDataHandler;
            RadarTcpClient.OnNavigationData += NavigationDataHandler;
            RadarTcpClient.OnConnectionChanged += ConnectionChangedHandler;

            //Connect to the radar on default IP address 192.168.0.1
            RadarTcpClient.Connect("10.77.2.211");

            //Sleep for 6 seconds to allow us to process some data
            Thread.Sleep(6000);

            Console.WriteLine("Test Client Stopping");
            //Send a message telling the radar to stop sending FFT Data
            RadarTcpClient.StopFftData();
            //RadarTcpClient.StopNavigationData();

            Thread.Sleep(1000);

            //Disconnect the client from the radar
            RadarTcpClient.Disconnect();

            Console.WriteLine("Test Client Stopped");
        }

        private static void NavigationDataHandler(object sender, NavigationDataEventArgs e)
        {
            Console.WriteLine($"Targets Found [{e.Data.NavigationReturns.Count}] First Target [{e.Data.NavigationReturns.First().Range}, {e.Data.NavigationReturns.First().Power / 10}]");
        }

        //FFT Data handler - handles incoming FFT Data messages
        private static void FftDataHandler(object sender, GenericEventArgs<FftData> fftEventArgs)
        {
            _PacketCount++;
            //Calculate the time difference between FFT Data messages and report the
            //rotation speed
            if (fftEventArgs.Payload.Message.Azimuth < _lastAzimuth)
            {
                var diff = DateTime.UtcNow - _lastRotationReset;
                Console.WriteLine($"FftDataHandler - Rotating @ [{ 1000.0 / diff.TotalMilliseconds:0.00}Hz] Packets [{_PacketCount}]");
                _lastRotationReset = DateTime.UtcNow;
                _PacketCount = 0;
            }

            _lastAzimuth = fftEventArgs.Payload.Message.Azimuth;
        }

        //Report the client connection status changes
        private static void ConnectionChangedHandler(object sender, ConnectionStateEventArgs connectionStateEventArgs)
        {
            Console.WriteLine("Connection Changed [{0}]", connectionStateEventArgs.State);
        }

        //Handle the configuration message. This is automatically sent after a client connects to the radar.
        //If you recevie the configuration data handler you can assume the client has successfully connected
        //or you can use the conenction chahnged status event
        private static void ConfigurationDataHandler(object sender, GenericEventArgs<TcpConfigurationDataMessage> configurationMessage)
        {
            //Display some of the configuration data received from the radar
            Console.WriteLine("ConfigurationDataHandler - Expected Rotation Rate [{0}Hz]", configurationMessage.Payload.RotationSpeed / 1000.0);
            Console.WriteLine("ConfigurationDataHandler - Range In Bins [{0}]", configurationMessage.Payload.RangeInBins);
            Console.WriteLine("ConfigurationDataHandler - Bin Size [{0}cm]", configurationMessage.Payload.BinSize / 10000.0);
            Console.WriteLine("ConfigurationDataHandler - Range In Metres [{0}m]", configurationMessage.Payload.BinSize / 10000.0 * configurationMessage.Payload.RangeInBins);
            Console.WriteLine("ConfigurationDataHandler - Azimuth Samples [{0}]", configurationMessage.Payload.AzimuthSamples);

            RadarTcpClient.SetNavigationGainAndOffset(1.0f, 0.0f);
            RadarTcpClient.SetNavigationThreshold(60 * 10);

            //Tell the radar to start sending FFT Data
            RadarTcpClient.StartFftData();
            //RadarTcpClient.StartNavigationData();
            _lastRotationReset = DateTime.UtcNow;
        }
    }
}
