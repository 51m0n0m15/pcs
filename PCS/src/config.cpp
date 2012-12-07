#include "config.h"


config::config(void)
{
}

config::~config(void)
{
}
//statische variablen muessen hier initialisiert werden, aus irgendeinem grund
int config::noise_level=0;
int config::plane_dist_threshold=100;
int config::min_cluster_size=500;
int config::radius_threshold=100;
int config::outliers_threshold=2;


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
		
		if(buffer[0] == 'n' && buffer[1] == 'l' && buffer[2] == 'v'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::noise_level = tmp;
		}

		if(buffer[0] == 'p' && buffer[1] == 'd' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::plane_dist_threshold = tmp;
		}

		if(buffer[0] == 'm' && buffer[1] == 'c' && buffer[2] == 's'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::min_cluster_size = tmp;
		}

		if(buffer[0] == 'r' && buffer[1] == 'a' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::radius_threshold = tmp;
		}

		if(buffer[0] == 'o' && buffer[1] == 'd' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::outliers_threshold = tmp;
		}

		

	}
 }
