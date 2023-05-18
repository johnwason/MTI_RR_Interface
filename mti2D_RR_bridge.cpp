
#include "mti2D_RR_bridge.h"

#include <boost/program_options.hpp>

namespace po = boost::program_options;
//This program provides an initial Robot Raconteur server for controlling MTI's 2D Laser Profiler (type: 8000-1066-002)
//Functions covers basic operations in this version. 

int main(int argc, char* argv[])
{
	po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("scanner-ip-address", po::value<std::string>(), "IP address of scanner")
		("scanner-port", po::value<std::string>(), "Port of scanner");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

	std::string device_ipaddr = "192.168.100.1";
	std::string device_port = "32001";

	if (vm.count("scanner-ip-address")) {
		device_ipaddr = vm["scanner-ip-address"].as<std::string>();
	}

	if (vm.count("scanner-port")) {
		device_port = vm["scanner-port"].as<std::string>();
	}
	
	//Initialize the MTI2D robot object
	boost::shared_ptr<MTI2D_impl> mti2d = boost::make_shared<MTI2D_impl>(device_ipaddr, device_port);

	if (mti2d->connected == true) {
		ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, "experimental.create2", 60830);

		//Register the MTI2D object as a service so that it can be connected to
		RobotRaconteurNode::s()->RegisterService("MTI2D", "mti2D_RR_interface", mti2d);

		cout << "MTI2D server started, connect with rr+tcp://localhost:" << node_setup.GetTcpTransport()->GetListenPort() << "/?service=MTI2D" << endl;

		//Stay open until shut down
		cout << "Press enter to quit" << endl;
		getchar();


		//Shutdown
		mti2d->Shutdown();

		//Shutdown the node. This must be called at program exit
		RobotRaconteurNode::s()->Shutdown();

		return 0;
	}
	else {
		cout << "Failed to start MTI2D server." << endl;

		//Stay open until shut down
		cout << "Press enter to quit" << endl;
		return 1;
	}
}


//Class for MTI2D_impl

//Initialize the serial port
MTI2D_impl::MTI2D_impl(string ip, string port) : MTI2D()
{
	connected = false;
	isDoubleSampling = false;
	
	Start(ip.c_str(), port.c_str());
	//Capture();

}


//Shutdown the serial port
void MTI2D_impl::Shutdown()
{

	boost::recursive_mutex::scoped_lock lock(global_lock);
	if (connected)
	{
		Disconnect();
		connected = false;
	}
}

MTI2D_impl::~MTI2D_impl()
{
	Shutdown();
}

void MTI2D_impl::Disconnect() {
	if (m_hEthernetScanner)
	{
		//activate signal to exit the ReadOut-Thread
		SetEvent(m_hCEthernetScannerThreadExitEvent);
		Sleep(1000);
		//close the ethernet connection to the scanner		
		if (m_hCEthernetScannerGetXZIThreadID != INVALID_HANDLE_VALUE)
		{
			CloseHandle(m_hCEthernetScannerGetXZIThreadID);
		}
		Sleep(2000);
		EthernetScanner_Disconnect(m_hEthernetScanner);
		m_hEthernetScanner = NULL;
	}
}

string MTI2D_impl::getPropertyValue(const string& propertyName)
{
	char chRetBuf[100];
	char *cstr = new char[propertyName.length() + 1];
	strcpy(cstr, propertyName.c_str());
	
	int iRetVal = EthernetScanner_ReadData(m_hEthernetScanner, cstr, chRetBuf, 100, 100);
	if (iRetVal == 1)
	{
		string strTemp(chRetBuf);
		delete[] cstr;
		return strTemp;
	}
	else
	{
		string strTemp = to_string(iRetVal);
		delete[] cstr;
		return strTemp;
	}
}

void MTI2D_impl::setExposureTime(const string& exposureTime) {

	int exp_t = atoi(exposureTime.c_str());

	if (exp_t>30 && exp_t<2000) {
		string strTemp = "SetExposureTime=" + exposureTime;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setAcquisitionLineTime(const string& acquisitionLineTime) {

	int acql_t = atoi(acquisitionLineTime.c_str());

	if (acql_t>165 && acql_t<10000) {
		string strTemp = "SetAcquisitionLineTime=" + acquisitionLineTime;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setLaserDeactivated(const string& isOff) {

	int isOff_i = atoi(isOff.c_str());

	if (isOff_i==0 || isOff_i==1) {
		string strTemp = "SetLaserDeactivated=" + isOff;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setSignalSelection(const string& signal) {

	int sig_i = atoi(signal.c_str());

	if (sig_i == 0 || sig_i == 1 || sig_i == 2 || sig_i ==3) {
		string strTemp = "SetSignalSelection=" + signal;
		char *cstr = new char[strTemp.length() + 1];
		strcpy(cstr, strTemp.c_str());
		EthernetScanner_WriteData(m_hEthernetScanner, cstr, strTemp.length());
		delete[] cstr;
	}
	return;
}

void MTI2D_impl::setIsDoubleSampling(const string& isDblSmpling) {

	int isDblSmpling_i = atoi(isDblSmpling.c_str());

	if (isDblSmpling_i == 0 || isDblSmpling_i == 1) {
		isDoubleSampling = isDblSmpling_i;
	}
	
	return;
}


//Capture line scan profile
LineProfilePtr MTI2D_impl::Capture()
{
	boost::recursive_mutex::scoped_lock lock(global_lock);

	LineProfilePtr profile(new LineProfile());


	if (connected) {

		int iPicCnt = 0;
		//EthernetScanner_GetXZIExtended: the Data are linearized
		int iEthernetScannerScanner1BufferValues = EthernetScanner_GetXZIExtended(this->m_hEthernetScanner,
			this->m_doEthernetScannerBufferX,
			this->m_doEthernetScannerBufferZ,
			this->m_iEthernetScannerBufferI,
			this->m_iEthernetScannerBufferPeakWidth,
			ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX,
			&this->m_uiEthernetScannerEncoder,
			&this->m_ucEthernetScannerUSRIO,
			0,
			NULL,
			0,
			&iPicCnt);

		profile->length = iEthernetScannerScanner1BufferValues;
		profile->X_data = AttachRRArrayCopy((double*)this->m_doEthernetScannerBufferX, iEthernetScannerScanner1BufferValues);
		profile->Z_data = AttachRRArrayCopy((double*)this->m_doEthernetScannerBufferZ, iEthernetScannerScanner1BufferValues);
		profile->I_data = AttachRRArrayCopy((int32_t*)this->m_iEthernetScannerBufferI, iEthernetScannerScanner1BufferValues);


		CString strTemp;
		strTemp.Format("%.4f %.4d %.4f %d",
			this->m_doEthernetScannerBufferX[iEthernetScannerScanner1BufferValues / 2],
			this->m_uiEthernetScannerEncoder,
			this->m_doEthernetScannerBufferZ[iEthernetScannerScanner1BufferValues / 2],
			this->m_iEthernetScannerBufferI[iEthernetScannerScanner1BufferValues / 2]);

		std::cout << strTemp << iPicCnt << iEthernetScannerScanner1BufferValues << std::endl;



		if (iEthernetScannerScanner1BufferValues == ETHERNETSCANNER_ERROR)
		{
			Sleep(1);
			
		}
		else {
			

		}
	}

	return profile;	
}


void MTI2D_impl::Start(const char* ipaddr, const char* nport)
{
	//get the current Path to the exe-File
	char lpPath[1024] = { 0 };
	GetModuleFileName(NULL, lpPath, sizeof(lpPath));
	m_structProgramm.FullPath.Format("%s", lpPath);
	m_structProgramm.DirectoryPath = m_structProgramm.FullPath.Left(m_structProgramm.FullPath.ReverseFind('\\') + 1);

	//get the current version of the DLL 
	unsigned char ucBuffer[1024];
	EthernetScanner_GetVersion(ucBuffer, 1024);
	CString strTemp;
	strTemp.Format("%s", ucBuffer);

	m_hEthernetScanner = NULL;

	//Default-Settings
	m_strScannerIP = ipaddr;
	m_strScannerPort = nport;
	m_strScannerTimeOut.Format("%u", 3000);

	m_strPropertyName = "Temperature";
	m_strCashTime.Format("%u", 100);


	//variables to control the ReadOut-Thread of the Scanner
	m_hCEthernetScannerThreadExitEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_hCEthernetScannerGetXZIThreadID = INVALID_HANDLE_VALUE;

	//initialize the view-class to show the scans


	m_dwEthernetScanner_ScanCnt = 0;

	m_iNumberProfilesToSaveCnt = 0;
	m_iNumberProfilesToSaveMax = 1;

	if (m_hEthernetScanner == NULL)
	{
		m_iPicCntErr = 0;
		CString strTemp;
		//start the connection to the Scanner
		m_hEthernetScanner = EthernetScanner_Connect(m_strScannerIP.GetBuffer(0),
			m_strScannerPort.GetBuffer(0),
			atoi(m_strScannerTimeOut));
		//if no valid value: ERROR
		if (m_hEthernetScanner == NULL)
		{
			std::cout << "Error: EthernetScanner_Connect: NULL-Pointer";
			std::cin.get();
			return;
		}

		//Wait for valid connection to scanner
		DWORD dwTimeOut = GetTickCount();
		int uiConnectionStatus = 0;

		while (uiConnectionStatus != ETHERNETSCANNER_TCPSCANNERCONNECTED)
		{
			//current state of the connection
			EthernetScanner_GetConnectStatus(m_hEthernetScanner, &uiConnectionStatus);
			//Detect the timeout
			if (GetTickCount() - dwTimeOut > 3000)
			{
				std::cout << "Error: EthernetScanner_Connect: Error in connection";
				m_hEthernetScanner = EthernetScanner_Disconnect(m_hEthernetScanner);
				std::cin.get();
				return;
			}
		}

		int iRes = -1;
		dwTimeOut = GetTickCount();
		while (GetTickCount() - dwTimeOut < SCANNER1TIMEOUT)
		{
			//the linearization table need to be transfered from the Scanner to the Host.
			//it takes some times
			//to be sure that the linearization table was received runs the "EthernetScanner_GetInfo"-function more times
			iRes = EthernetScanner_GetInfo(m_hEthernetScanner, m_chEthernetScannerInfo, ETHERNETSCANNER_GETINFOSIZEMAX, "text");
			if (iRes > 0)
			{
				break;
			}

		}
		if (iRes < 0)
		{
			std::cout << "Error in linearisation data!!!";
			std::cin.get();
		}
		iRes = EthernetScanner_GetInfo(m_hEthernetScanner, m_chEthernetScannerInfo, ETHERNETSCANNER_GETINFOSIZEMAX, "xml");
		if (iRes > 0)
		{		
			connected = true;
			m_lineProfile = new LineProfile();

			//start the ReadOut-Thread
			DWORD lpThreadId;
			m_hCEthernetScannerGetXZIThreadID = CreateThread(NULL,
				NULL,
				(LPTHREAD_START_ROUTINE)ThreadEthernetScannerGetXZI,
				(LPVOID)this, //Pointer to this Class. to access the Class-Variables from the Thread
				0,
				&lpThreadId);
			if (m_hCEthernetScannerGetXZIThreadID)
			{
			}
		}
		else
		{
			//get XML Error
		}

		/*std::cout << "Reached the end of 'Start', enter to continue" << std::endl;
		std::cin.get();*/
	}
}

/*
ReadOut-Thread of Scans
runs til the m_hCEthernetScannerScanner1ThreadExitEvent-Event not signaled
*/
DWORD WINAPI MTI2D_impl::ThreadEthernetScannerGetXZI(LPVOID lpParameter)
{
	MTI2D_impl *pApp = (MTI2D_impl *)lpParameter;

	bool bRun = TRUE;

	DWORD dwScanner1FrequnzTimer = GetTickCount();
	int iScanner1ShowFrequency = GetTickCount();
	int iScanner1ShowScan = GetTickCount();

	CString strTemp;

	bool bShowOnce = true;// false;
	int uiConnectionStatus = ETHERNETSCANNER_TCPSCANNERDISCONNECTED;

	int iFrameCntTemp = 0;

	/*std::cout << "Entering infinite loop, enter to continue" << std::endl;
	std::cin.get();*/

	//chech the valid value of the Scanner-Handle: NULL is not valid, otherwise is OK
	while (pApp->m_hEthernetScanner)
	{
		//current state of the connection
		EthernetScanner_GetConnectStatus(pApp->m_hEthernetScanner, &uiConnectionStatus);

		if (uiConnectionStatus == ETHERNETSCANNER_TCPSCANNERCONNECTED)
		{

			int iPicCnt = 0;
			//EthernetScanner_GetXZIExtended: the Data are linearized
			int len_arr = ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX;
			for (int i = 0; i < len_arr; i++) {
				pApp->m_doEthernetScannerBufferX_prev[i] = pApp->m_doEthernetScannerBufferX[i];
				pApp->m_doEthernetScannerBufferZ_prev[i] = pApp->m_doEthernetScannerBufferZ[i];
				pApp->m_iEthernetScannerBufferI_prev[i] = pApp->m_iEthernetScannerBufferI[i];
			}
			int iEthernetScannerScanner1BufferValues = EthernetScanner_GetXZIExtended(pApp->m_hEthernetScanner,
				pApp->m_doEthernetScannerBufferX,
				pApp->m_doEthernetScannerBufferZ,
				pApp->m_iEthernetScannerBufferI,
				pApp->m_iEthernetScannerBufferPeakWidth,
				ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX,
				&pApp->m_uiEthernetScannerEncoder,
				&pApp->m_ucEthernetScannerUSRIO,
				0,
				NULL,
				0,
				&iPicCnt);

			if (iEthernetScannerScanner1BufferValues == ETHERNETSCANNER_ERROR)
			{
				Sleep(1);
				continue;
			}

			if (iPicCnt - pApp->m_usPicCnt != 1)
			{
				if ((iPicCnt == 0) && (pApp->m_usPicCnt == 0xFFFF))
				{
				}
				else
				{
					pApp->m_iPicCntErr++;
				}
			}
			pApp->m_usPicCnt = iPicCnt;

			LineProfilePtr profile(new LineProfile());
			
			if (pApp->isDoubleSampling) {
				double m_doEthernetScannerBufferX_temp[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];

				double m_doEthernetScannerBufferZ_temp[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];

				int m_iEthernetScannerBufferI_temp[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];				

				for (int i = 0; i < len_arr; i++) {
					m_doEthernetScannerBufferX_temp[i] = (pApp->m_doEthernetScannerBufferX_prev[i] + pApp->m_doEthernetScannerBufferX[i])/2.0;
					m_doEthernetScannerBufferZ_temp[i] = (pApp->m_doEthernetScannerBufferZ_prev[i] + pApp->m_doEthernetScannerBufferZ[i])/2.0;
					m_iEthernetScannerBufferI_temp[i] = (pApp->m_iEthernetScannerBufferI_prev[i] + pApp->m_iEthernetScannerBufferI[i])/2;
				}

				profile->length = iEthernetScannerScanner1BufferValues;
				profile->X_data = AttachRRArrayCopy((double*)m_doEthernetScannerBufferX_temp, iEthernetScannerScanner1BufferValues);
				profile->Z_data = AttachRRArrayCopy((double*)m_doEthernetScannerBufferZ_temp, iEthernetScannerScanner1BufferValues);
				profile->I_data = AttachRRArrayCopy((int32_t*)m_iEthernetScannerBufferI_temp, iEthernetScannerScanner1BufferValues);				
			}
			else {
				profile->length = iEthernetScannerScanner1BufferValues;
				profile->X_data = AttachRRArrayCopy((double*)pApp->m_doEthernetScannerBufferX, iEthernetScannerScanner1BufferValues);
				profile->Z_data = AttachRRArrayCopy((double*)pApp->m_doEthernetScannerBufferZ, iEthernetScannerScanner1BufferValues);
				profile->I_data = AttachRRArrayCopy((int32_t*)pApp->m_iEthernetScannerBufferI, iEthernetScannerScanner1BufferValues);
			}
			

			pApp->set_lineProfile(profile);

			//if the scan data was received: do anything
			if (iEthernetScannerScanner1BufferValues)
			{
				///count the frequency
				pApp->m_dwEthernetScanner_Frequenz++;
				pApp->m_dwEthernetScanner_ScanCnt++;

				//with higher frequency or slower PC-CPU the view of scans can be disabled
				if (pApp->m_bViewDisable == false)
				{
					//decrease or disable the view of the scans if the scanner frequency go high!
					if (GetTickCount() - iScanner1ShowScan > 100)
					{
						iScanner1ShowScan = GetTickCount();

					}
				}
			}
		}
		//exit the thread if the signal comes
		if (WaitForSingleObject(pApp->m_hCEthernetScannerThreadExitEvent, 0) == WAIT_OBJECT_0)
		{
			bRun = FALSE;
		}

	}
	return (0);
}



//Set the lineProfile property
void MTI2D_impl::set_lineProfile(const LineProfilePtr& profile)
{
	m_lineProfile->length = profile->length;
	m_lineProfile->X_data = profile->X_data;
	m_lineProfile->Z_data = profile->Z_data;
	m_lineProfile->I_data = profile->I_data;	
}

//Return the DistanceTraveled property
LineProfilePtr MTI2D_impl::get_lineProfile()
{
	return m_lineProfile;
}


boost::recursive_mutex global_lock;