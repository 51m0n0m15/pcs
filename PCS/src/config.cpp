#include "config.h"


config::config(void)
{
}

config::~config(void)
{
}
//statische variablen muessen hier initialisiert werden, aus irgendeinem grund
int config::filter_leaf_size=200;
int config::plane_dist_threshold=100;
int config::cluster_dist_threshold=100;
int config::min_cluster_size=500;
float config::normal_diff_threshold=0.1;
int config::curvature_threshold=1;
int config::normal_est_k=50;
int config::region_growing_k=30;

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
			config::filter_leaf_size = tmp;
		}

		if(buffer[0] == 'p' && buffer[1] == 'd' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::plane_dist_threshold = tmp;
		}

		if(buffer[0] == 'c' && buffer[1] == 'd' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::cluster_dist_threshold = tmp;
		}

		if(buffer[0] == 'm' && buffer[1] == 'c' && buffer[2] == 's'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::min_cluster_size = tmp;
		}

		if(buffer[0] == 'n' && buffer[1] == 'd' && buffer[2] == 't'){
			float tmp;
			fscanf(configFile, "%f", &tmp);
			config::normal_diff_threshold = tmp;
		}

		if(buffer[0] == 'c' && buffer[1] == 'u' && buffer[2] == 't'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::curvature_threshold = tmp;
		}

		if(buffer[0] == 'n' && buffer[1] == 'e' && buffer[2] == 'k'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::normal_est_k = tmp;
		}

		if(buffer[0] == 'r' && buffer[1] == 'g' && buffer[2] == 'k'){
			int tmp;
			fscanf(configFile, "%d", &tmp);
			config::region_growing_k = tmp;
		}

	}
 }
