#include "stdafx.h"
#include "Settings.h"

Settings::~Settings(void)
{
}
Settings::Settings()
{

}
//Kalibrierung
bool Settings::globalDebugshowPictures=0;
bool Settings::saveNotTestedPictures=1;
int Settings::cameraresWidth=1280;
int Settings::cameraresHight=1024;
int Settings::featureFindMinHessian=600;//17 
float Settings::skalierfaktor=1/16;
string Settings::calibrateFilenameXML="Settings.xml";
string Settings::calibrateFilenamePictures="Pictures\\";
string Settings::calibrateFilenamePicturesCamA="KameraA\\";
string Settings::calibrateFilenamePicturesCamB="KameraB\\";
string Settings::calibrateFilenamePicturesNotTested="NichtGetestet\\";

//BilderAufnahme
int Settings::webcamFirstWebcam=1;
bool Settings::webcamShowChessboard=0;
bool Settings::webcamShowFeatures=0;
bool Settings::webcamFlippedCams[10]={1,0,0,0,0,0,0,0,0,0};
bool Settings::webcamFlippedCamsContain(int number)
{
	return Settings::webcamFlippedCams[number];
}
void Settings::uiChangeSettings()
{
	while(1)
	{
#pragma region Text
		std::cout<<"-------------------------------------------------------"<<endl;
		std::cout<<"Bilder werden bei der Eingabe angezeigt: "<<Settings::globalDebugshowPictures<<endl;
		std::cout<<"Schachbrett in Webcam erkennen: "<<Settings::webcamShowChessboard<<endl;
		std::cout<<"Erste genutzte Kamera: "<<Settings::webcamFirstWebcam<<endl;
		std::cout<<"File zum speichern der SettingsXML: "<<Settings::calibrateFilenameXML<<endl;
		std::cout<<"File zum speichern der Bilder: "<<Settings::calibrateFilenamePictures<<endl;
		std::cout<<"Features werden angezeigt: "<<Settings::webcamShowFeatures<<endl;
		std::cout<<"-------------------------------------------------------"<<endl;
		std::cout<<"[1] Sollen Bilder angezeigt werden [DEBUG]"<<endl;
		std::cout<<"[2] Soll bei Kameras erkannte Schachbretter angezeigt werden [LAG]"<<endl;
		std::cout<<"[3] Erste Kamera festelegen[sperren einer Webcam]"<<endl;
		std::cout<<"[4] Aendern des Speicherortes XML"<<endl;
		std::cout<<"[5] Soll bei der Kamera erkannte Features angezeigt werden?"<<endl;
		std::cout<<"[6] Kameras umdrehen"<<endl;
		std::cout<<"[7] Speicherort der Bidler aendern"<<endl;
		std::cout<<"[sonst] Abbrechen"<<endl;
#pragma endregion
		int input=0;
		cin>>input;
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(),'\n');

		switch (input)
		{
		case 1: Settings::globalDebugshowPictures=Settings::globalDebugshowPictures?false:true;break;
		case 2: Settings::webcamShowChessboard=Settings::webcamShowChessboard?false:true;break;
		case 3:
			{
				cout<<"Input: ";
				cin>>input;
				cin.clear();
				cin.ignore(numeric_limits<streamsize>::max(),'\n');
				Settings::webcamFirstWebcam=input;
				break;
			}
		case 4:
			{
				string inputString;
				cin>>inputString;
				cin.clear();
				cin.ignore(numeric_limits<streamsize>::max(),'\n');
				Settings::calibrateFilenameXML=inputString;
			}
		case 5: Settings::webcamShowFeatures=Settings::webcamShowFeatures?false:true;break;
		case 6:
			{while(1){

				std::cout<<"-------------------------------------------------------"<<endl;
				for (int i = 0; i<9;i++)
					std::cout<<"|"<<Settings::webcamFlippedCams[i];
				std::cout<<endl;
				std::cout<<"-------------------------------------------------------"<<endl;
				cout<<"[x] Kamera aendern?"<<endl;
				cout<<"[xx] abbrechen?"<<endl;
				int inputinteger;
				cin>>inputinteger;
				cin.clear();
				cin.ignore(numeric_limits<streamsize>::max(),'\n');
				if (inputinteger>9||inputinteger<0)break;
				else
				{
					Settings::webcamFlippedCams[inputinteger]=
						Settings::webcamFlippedCams[inputinteger]?false:true;
				}

			}break;}
		case 7:
			{
				string inputString;
				cin>>inputString;
				cin.clear();
				cin.ignore(numeric_limits<streamsize>::max(),'\n');
				Settings::calibrateFilenamePictures=inputString;
			}

		default:return;
		}
	}
}





