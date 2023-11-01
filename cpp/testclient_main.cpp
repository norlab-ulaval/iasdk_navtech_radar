/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#include "radarclient.h"

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif

using namespace Navtech;

uint16_t _lastAzimuth = 0;
uint16_t _packetCount = 0;
uint64_t _lastRotationReset = Helpers::Now();
RadarClientPtr_t _radarClient;

void FFTDataHandler(const FFTDataPtr_t& data)
{
	_packetCount++;
	if(data->Azimuth < _lastAzimuth) {
		auto diff = Helpers::Now() - _lastRotationReset;
		//Helpers::Log("FFTDataHandler - Rotating @ [" + std::to_string(1000.0 / diff) + "Hz] Packets [" + std::to_string(_packetCount) + "]");
		_lastRotationReset = Helpers::Now();
		_packetCount = 0;
		
	}
	std::cout << data->Azimuth<<"\n";
	_lastAzimuth = data->Azimuth;
}

void ConfigurationDataHandler(const ConfigurationDataPtr_t& data)
{
	Helpers::Log("ConfigurationDataHandler - Expected Rotation Rate [" + std::to_string(data->ExpectedRotationRate) + "Hz]");
	Helpers::Log("ConfigurationDataHandler - Range In Bins [" + std::to_string(data->RangeInBins) + "]");
	Helpers::Log("ConfigurationDataHandler - Bin Size [" + std::to_string(data->BinSize / 10000.0) + "cm]");
	Helpers::Log("ConfigurationDataHandler - Range In Metres [" + std::to_string((data->BinSize / 10000.0) * data->RangeInBins) + "m]");
	Helpers::Log("ConfigurationDataHandler - Azimuth Samples [" + std::to_string(data->AzimuthSamples) + "]");
	
	_packetCount = 0;
	_lastAzimuth = 0;
	_radarClient->SetNavigationGainAndOffset(1.0f, 0.0f);
	_radarClient->SetNavigationThreshold(60 * 10);

	_radarClient->StartFFTData();
	_radarClient->StartNavigationData();
}

void NavigationDataHandler(const NavigationDataPtr_t& data)
{	
	auto firstRange = std::get<0>(data->Peaks[0]);
	auto firstPower = std::get<1>(data->Peaks[0]);
	//Helpers::Log("NavigationDataHandler - First Target [" + std::to_string(firstRange) + "] [" + std::to_string(firstPower / 10) + "]");	
	
}

int32_t main(int32_t argc, char** argv)
{
#ifdef _WIN32
	WSADATA wsaData;
	auto err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0) {
		Helpers::Log("Tracker - WSAStartup failed with error [" + std::to_string(err) + "]");
		return EXIT_FAILURE;
	}
#endif
	
	Helpers::Log("Test Client Starting");
	
	_radarClient = std::make_shared<RadarClient>("10.77.2.170");
	_radarClient->SetFFTDataCallback(std::bind(&FFTDataHandler, std::placeholders::_1));
	_radarClient->SetConfigurationDataCallback(std::bind(&ConfigurationDataHandler, std::placeholders::_1));
	_radarClient->SetNavigationDataCallback(std::bind(&NavigationDataHandler, std::placeholders::_1));	
	_radarClient->Start();
	
	std::this_thread::sleep_for(std::chrono::milliseconds(6000));

	Helpers::Log("Test Client Stopping");
	//_radarClient->StopNavigationData();
	_radarClient->StopFFTData();

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	_radarClient->SetNavigationDataCallback();
	_radarClient->SetConfigurationDataCallback();
	_radarClient->SetFFTDataCallback();
	
	_radarClient->Stop();

	Helpers::Log("Test Client Stopped");
	
	return 0;
}
