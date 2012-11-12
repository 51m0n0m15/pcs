#include "config.h"


config::config(void)
{
}

config::~config(void)
{
}
//statische variablen muessen hier initialisiert werden, aus irgendeinem grund
int config::filterLeafSize=200;
int config::planeDistThreshold=100;
int config::clusterDistThreshold=100;
int config::minClusterSize=500;

void config::readConfig(){

	FILE *configFile;
	char buffer[64];

	char *config = "config.cfg";
	configFile = fopen(config, "r");

	if(!configFile){
		cout << "Error opening file " << config << "; using default values." << endl;
		return;
	}


	while (fscanf(configFile, "%s", buffer) != EOF){
		
		if(buffer[0] == 'f' && buffer[1] == 'l' && buffer[2] == 's'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::filterLeafSize = tmp;
		}

		if(buffer[0] == 'p' && buffer[1] == 'd' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::planeDistThreshold = tmp;
		}

		if(buffer[0] == 'c' && buffer[1] == 'd' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::clusterDistThreshold = tmp;
		}

		if(buffer[0] == 'm' && buffer[1] == 'c' && buffer[2] == 's'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::minClusterSize = tmp;
		}

	}
 }
