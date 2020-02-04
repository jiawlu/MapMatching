// MapMatching.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <list> 
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "CSVParser.h"
#include <functional>
#include <stdio.h>   
#include <tchar.h>
#include <windows.h>
#include <math.h>


//code for map matching

#define _MAX_LABEL_COST 99999
#define _MAX_PROCESSOR_SIZE 64
#define _MAX_STATES 1

#define _MAX_NUMBER_OF_TIME_INTERVALS 10000		//1440

#define _MAX_TAU_SIZE 1440 

int path_data_array[300];  // 300 for max path path size

TCHAR g_DTASettingFileName[_MAX_PATH] = _T("./DTASettings.txt");


extern void g_OutAgentCSVFile_FromSimulation();

template <typename T>
T **Allocate2DDynamicArray(int nRows, int nCols)
{
	T **dynamicArray;

	dynamicArray = new T*[nRows];

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new T[nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}
	}

	return dynamicArray;
}

template <typename T>
void Deallocate2DDynamicArray(T** dArray, int nRows)
{
	for (int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete[] dArray;
}

template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}

	for (int x = 0; x < nX; x++)
	{
		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}
		}
	}

	return dynamicArray;
}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;
}


using namespace std;

TCHAR g_SettingFileName[_MAX_PATH] = _T("./Settings.txt");

FILE* g_pFileDebugLog = NULL;
FILE* g_pFileOutputLog = NULL;
FILE* g_pTSViewOutput = NULL;
FILE* g_pNGSIMOuputLog = NULL;

int g_right_block_time = 2;
int g_left_block_time = 3;
int g_dp_algorithm_debug_flag = 0;

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_zones = 0;

int g_shortest_path_debugging_flag = 0;
int g_number_of_agents;
int g_NumberOfIterations = 1;
int g_TrafficFlowModel = 1;
int g_generate_agent_csv = 1;
int g_generate_link_TDMOE_csv = 0;
int g_number_of_signals = 0;
int g_number_of_threads = 1;
int max_number_of_links_per_path = 50;

#define _MAX_ZONE_SIZE 4000
double g_loading_multiplier = 0.66666666;
double g_ODME_adjusment_step_size = 0.01;
int g_max_number_of_agents = 5000000;
double g_OD_loading_multiplier[_MAX_ZONE_SIZE][_MAX_ZONE_SIZE];

int g_number_of_simulation_intervals = 12500;  // min
int g_extra_time = 1000;

float g_number_of_seconds_per_interval = 0.2;
int g_number_of_intervals_per_min = 60 / g_number_of_seconds_per_interval;
int g_number_of_simulation_minutes = 100;
int g_vehicle_waiting_time_interval = 1;

int g_Simulation_StartTimeInMin = 9999;
int g_Simulation_EndTimeInMin = 0;
int g_Post_Simulation_DurationInMin = 1;
int g_waiting_time_inverval = 1;

vector<int> g_partial_schedule_agent_sequence; //given the partial schedule sequence for the second stage

int m_ListFront;
int m_ListTail;

// 6 seconds per interval
// 3600 -> 6
// 1800 -> 3
// 900 -> 1.5

std::map<int, int> g_link_key_to_seq_no_map;  // hush table, map key to internal link sequence no.
std::map<int, int> g_internal_link_no_map;  // hash table
std::map<int, int> g_external_link_id_map;  // hash table

std::map<int, int> g_internal_agent_no_map; // map exteranl agent id to internal agent no.
std::map<int, int> g_external_agent_id_map; // map internal agent no. to external agent id


//extern double WELLRNG512a(void);
//extern void InitWELLRNG512a(unsigned int *init);
//
//double g_GetRandomRatio()
//{
//	return WELLRNG512a();
//}
// end of random number seeds

typedef struct
{
	int agent_id;
	int agent_type;
	int from_origin_node_id;
	int to_destination_node_id;
	int vehicle_seat_capacity;
	int PCE_factor;
	float preferred_departure_time;
	float travel_time_in_min;
	float arrival_time_in_min;

	int fixed_path_flag;
	int number_of_nodes;
	int path_index;
	//path_node_sequence, path_time_sequence

} struct_AgentInfo_Header;

typedef struct
{
	int node_id;      //external node number 
	int zone_id;

} struct_NodeInfo_Header;

typedef struct
{
	int from_node_id;
	int to_node_id;
	int link_type;
	int service_type;
	float length;
	float speed_limit;
	float BPR_alpha_term;
	float BPR_beta_term;
	int number_of_lanes;
	int lane_cap;
	int jam_density;
} struct_LinkInfo_Header;

//mfd
int g_TAU;

std::map<int, int> g_internal_node_seq_no_map;  // hush table, map external node number to internal node sequence no. 
std::map<int, int> g_internal_node_seq_no_to_node_id_map;  // hush table, map external node number to internal node sequence no. 

std::map<int, int> g_internal_signal_seq_no_map;
std::map<int, int> g_internal_signal_seq_no_to_node_id_map;

long g_GetLinkSeqNo(int from_node_no, int to_node_no)
{
	if (g_internal_node_seq_no_map.find(from_node_no) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	if (g_internal_node_seq_no_map.find(to_node_no) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	int from_node_seq_no = g_internal_node_seq_no_map[from_node_no];
	int to_node_seq_no = g_internal_node_seq_no_map[to_node_no];

	long link_key = from_node_seq_no * 100000 + to_node_seq_no;

	if (g_link_key_to_seq_no_map.find(link_key) != g_link_key_to_seq_no_map.end())
		return g_link_key_to_seq_no_map[link_key];
	else
		return -1;
}

class CLink
{
public:
	CLink()  // construction 
	{
		lane_cap = 1000;
		service_type = 0;
		cost = 0;
		BPR_alpha_term = 0.15f;
		BPR_beta_term = 4.0f;
		link_capacity = 1000;
		jam_density = 100;
		free_flow_travel_time_in_min = 1;
		flow_volume = 0;
		number_of_lanes = 1;
		// mfd
		mfd_zone_id = 0;

		speed_limit = 10;
	}

	~CLink()
	{

	}

	std::list<int>  m_waiting_traveler_queue;

	int state_travel_time_vector[_MAX_STATES];

	int** state_dependent_travel_time_matrix;  // travel time in terms of simulation/optimization time interval
	float* time_dependent_flow_volume_vector;
	float** state_dependent_time_dependent_LR_multiplier_matrix;

	// (i,j), from t at mode m--> 
	// method 1: determined by train schedule from input_agent.csv, path_schedule_time_sequence
	// method 2: on pick up links, from given time t, find the next available k trains at time t', TT= t'-t
	// method 3: on freeway links, at given time t, use state_travel_time_vector from input_link.csv to generate t'
	// method 4: on waiting links: departure from home link and to final destination links, provide a feasible range of waiting time at the same mode of 0

	void Setup_State_Dependent_Data_Matrix()
	{
		state_dependent_travel_time_matrix = Allocate2DDynamicArray<int>(g_number_of_simulation_intervals, _MAX_STATES);

		state_dependent_time_dependent_LR_multiplier_matrix = Allocate2DDynamicArray<float>(g_number_of_simulation_intervals, _MAX_STATES);

		time_dependent_flow_volume_vector = new float[g_number_of_simulation_intervals];

		for (int t = 0; t < g_number_of_simulation_intervals; t++)
		{
			time_dependent_flow_volume_vector[t] = 0;

			for (int s = 0; s < _MAX_STATES; s++)
			{
				state_dependent_travel_time_matrix[t][s] = 1;  //assume simulation time interval as free-flow travel time per cell 
				state_dependent_time_dependent_LR_multiplier_matrix[t][s] = 0;
			}
		}
	}


	int link_id;
	int link_type; //0 main track; 1 other tracks, siding tracks, cross-over tracks
	int lane_id;
	string name;

	int shp_id;
	int from_node_id;   // external node numbers, 
	int to_node_id;
	int direction; // 1 is upward direction; 2 is downward direction
	double local_distance;

	int link_seq_no;
	int from_node_seq_no;  // starting from 0, sequential numbers 
	int to_node_seq_no;
	float cost;

	float free_flow_travel_time_in_min;
	int free_flow_travel_time_in_simu_interval;
	float length;
	float speed_limit;
	int number_of_lanes;

	int lane_capacity;
	int lane_cap;
	int type;

	int service_type; // 0: moving, -1: drop off, +1, pick up
	float link_capacity;  // per hour
	float link_capacity_per_min;  // per hour

	float jam_density;
	float flow_volume;
	float travel_time;
	float BPR_alpha_term;
	float BPR_beta_term;

	double x;
	double y;
	double z;
	double local_y;

	// mfd
	int mfd_zone_id;
};

class CNode
{
public:
	CNode()
	{
		zone_id = 0;
		accessible_node_count = 0;
		bOriginNode_ForAgents = false;
		m_SeqNo4OriginNode = -1;
	}

	void Setup_Node_State_Matrix()
	{
		node_state_matrix = new int[g_number_of_simulation_intervals];

		for (int t = 0; t < g_number_of_simulation_intervals; t++)
		{
			node_state_matrix[t] = 1;

		}
	}

	int accessible_node_count;

	int node_seq_no;  // sequence number
	int node_id;      //external node number
	string name;
	int zone_id;
	double x;
	double y;
	double z;

	bool bOriginNode_ForAgents;
	int m_SeqNo4OriginNode;

	int* node_state_matrix;

	std::vector<CLink> m_outgoing_node_vector;
};

std::vector<CNode> g_node_vector;
std::vector<CLink> g_link_vector;

int g_GetLinkSeqNoByNodeSeq(int from_node_no, int to_node_no)
{
	for (int i = 0; i < g_link_vector.size(); i++)
	{
		if (g_link_vector[i].from_node_seq_no == from_node_no && g_link_vector[i].to_node_seq_no == to_node_no)
		{
			return g_link_vector[i].link_seq_no;
		}
	}

	return -1;
}


class CGPSPoint
{
public:
	CGPSPoint() {}
	~CGPSPoint() {}

public:
	double x;
	double y;
	double time_interval_no;

};


class CAgent
{
public:
	CAgent()
	{
		Reset();

	}

	void Reset()
	{
		PCE_factor = 1.0;
		path_cost = 0;
		m_random_seed = 1;

		m_bLoaded = false;
		m_bComplete = false;
		m_SimLinkSequenceNoInPath = -1;

		m_ArrivalTime = 0;
		m_TripTime = 0;
		m_TripFFTT = 0;
		m_BufferWaitingTime = 0;
		m_TollDollarCost = 0;
		Energy = 0;
		CO2 = 0;
		NOX = 0;
		CO = 0;
		HC = 0;
		m_Delay = 0;

		within_link_driving_distance = 0;
		m_TimeToRetrieveInfo = 99999;
	}


	int agent_id;
	unsigned int m_random_seed;
	float m_TimeToRetrieveInfo;
	float m_LeavingTimeFromLoadingBuffer;

	bool m_bComplete;
	float m_TollDollarCost;

	float within_link_driving_distance;

	int origin_node_seq_no;
	int destination_node_seq_no;
	float departure_time_in_min;

	float m_Distance;

	float m_ArrivalTime;
	float m_TripTime;
	float m_TripFFTT;
	float m_BufferWaitingTime;
	float m_TravelTime;
	float m_EstimatedTravelTime;
	float m_Delay;

	float Energy, CO2, NOX, CO, HC, PM, PM2_5;


	float PCE_factor;  // passenger car equivalent : bus = 3
	float path_cost;
	std::vector<int> path_link_seq_no_vector;
	std::vector<int> path_node_seq_no_vector;
	std::vector<float> path_node_timestamp_vector;
	std::vector<int> path_NFD_zone_no_per_link_vector;

	std::vector<int> path_node_id_vector;	//jiawei
	std::vector<int> path_timestamp_vector;	//jiawei

	std::vector<CGPSPoint> m_GPSPointVector; //jiawei
	std::map<int, int> time_stamp_gps_point_no_map; // map time stamp to gps point no

	// for simulation 
	bool m_bLoaded;
	int m_SimLinkSequenceNoInPath;

	int origin_node_id;//luchao
	int destination_node_id;//luchao
	std::vector<CNode*> path_node_sqe_vector;//luchao
	double totaltraveldistance;//luchao
	std::vector<string> RoadNameVector;
	bool GetSpace_Time_State_Path = true;

};


class CAgentElement
{
public:
	CAgentElement()
	{
		bActive = true;
	}


	int agent_no;
	bool bActive;
};

vector<CAgent> g_agent_vector;
std::map<int, int> g_map_agent_id_to_agent_vector_seq_no;

void g_ProgramStop()
{

	cout << "AgentLite Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};



void g_ReadInputData()
{
	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0

	if (true)	//g_ReadInputBinFile() == 0
	{
		// step 1: read node file 
		CCSVParser parser;

		if (parser.OpenCSVFile("input_node.csv", true))
		{
			int internal_node_seq_no = 0;
			double x, y, z;

			std::map<int, int> node_id_map;

			while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
			{
				string name;

				int node_type;
				int node_id;

				if (parser.GetValueByFieldName("node_id", node_id) == false)
					continue;

				if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
				{
					continue; //has been defined
				}

				g_internal_node_seq_no_map[node_id] = internal_node_seq_no;
				g_internal_node_seq_no_to_node_id_map[internal_node_seq_no] = node_id;

				parser.GetValueByFieldName("x", x, false);
				parser.GetValueByFieldName("y", y, false);
				parser.GetValueByFieldName("z", z, false);
				parser.GetValueByFieldName("name", name);

				CNode node;  // create a node object
				node.node_id = node_id;
				node.node_seq_no = internal_node_seq_no;
				node.name = name;
				parser.GetValueByFieldName("zone_id", node.zone_id);

				node.x = x;
				node.y = y;
				node.z = z;
				internal_node_seq_no++;

				g_node_vector.push_back(node);  // push it to the global node vector

				g_number_of_nodes++;
				if (g_number_of_nodes % 1000 == 0)
					cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
			}

			cout << "number of nodes = " << g_number_of_nodes << endl;

			fprintf(g_pFileOutputLog, "number of nodes =,%d\n", g_number_of_nodes);
			parser.CloseCSVFile();
		}

		// step 2: read link file 

		CCSVParser parser_link;

		if (parser_link.OpenCSVFile("input_link.csv", true))
		{
			int previous_link_id_origin = 0;
			int current_link_id_origin = 0;
			float current_local_distance = 0;

			while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
			{
				CLink link;  // create a link object

				if (parser_link.GetValueByFieldName("link_id", link.link_id) == false)
					continue;
				if (parser_link.GetValueByFieldName("from_node_id", link.from_node_id) == false)
					continue;
				if (parser_link.GetValueByFieldName("to_node_id", link.to_node_id) == false)
					continue;

				parser_link.GetValueByFieldName("link_type", link.link_type);
				parser_link.GetValueByFieldName("name", link.name);
				parser_link.GetValueByFieldName("direction", link.direction);
				parser_link.GetValueByFieldName("lane_id", link.lane_id);

				// add the to node id into the outbound (adjacent) node list
				link.shp_id = link.link_id;
				int internal_from_node_seq_no = g_internal_node_seq_no_map[link.from_node_id];  // map external node number to internal node seq no. 
				int internal_to_node_seq_no = g_internal_node_seq_no_map[link.to_node_id];

				link.from_node_seq_no = internal_from_node_seq_no;
				link.to_node_seq_no = internal_to_node_seq_no;
				link.link_seq_no = g_number_of_links;

				parser_link.GetValueByFieldName("local_y", link.local_y);

				g_internal_link_no_map[link.link_id] = link.link_seq_no;
				g_external_link_id_map[link.link_seq_no] = link.link_id;

				parser_link.GetValueByFieldName("link_type", link.type);
				parser_link.GetValueByFieldName("service_type", link.service_type, false);

				float length = 1; // km or mile

				parser_link.GetValueByFieldName("length", length);
				parser_link.GetValueByFieldName("speed_limit", link.speed_limit);

				parser_link.GetValueByFieldName("BPR_alpha_term", link.BPR_alpha_term);
				parser_link.GetValueByFieldName("BPR_beta_term", link.BPR_beta_term);
				int number_of_lanes = 1;

				parser_link.GetValueByFieldName("number_of_lanes", link.number_of_lanes);
				parser_link.GetValueByFieldName("lane_cap", link.lane_cap);
				parser_link.GetValueByFieldName("jam_density", link.jam_density);

				link.link_capacity = link.lane_cap* number_of_lanes;
				link.free_flow_travel_time_in_min = 60 * length / link.speed_limit;

				parser_link.GetValueByFieldName("travel_time_s0", link.state_travel_time_vector[0]);
				parser_link.GetValueByFieldName("travel_time_s1", link.state_travel_time_vector[1]);
				parser_link.GetValueByFieldName("travel_time_s2", link.state_travel_time_vector[2]);
				parser_link.GetValueByFieldName("travel_time_s3", link.state_travel_time_vector[3]);
				parser_link.GetValueByFieldName("travel_time_s4", link.state_travel_time_vector[4]);
				parser_link.GetValueByFieldName("travel_time_s5", link.state_travel_time_vector[5]);
				parser_link.GetValueByFieldName("travel_time_s6", link.state_travel_time_vector[6]);

				link.length = length;
				link.cost = length / link.speed_limit * 60; // min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate delay 

				link.x = (g_node_vector[link.from_node_seq_no].x + g_node_vector[link.to_node_seq_no].x) / 2.0;
				link.y = (g_node_vector[link.from_node_seq_no].y + g_node_vector[link.to_node_seq_no].y) / 2.0;
				link.z = (g_node_vector[link.from_node_seq_no].z + g_node_vector[link.to_node_seq_no].z) / 2.0;

				g_node_vector[internal_from_node_seq_no].m_outgoing_node_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link

				current_link_id_origin = link.shp_id;

				if (g_number_of_links == 0)
				{
					previous_link_id_origin = link.shp_id;
				}
				else
				{
					previous_link_id_origin = g_link_vector[g_number_of_links - 1].shp_id;
				}

				if (previous_link_id_origin == current_link_id_origin)
				{
					if (g_number_of_links == 0)
					{
						current_local_distance = link.length / 2.0f;
					}
					else
					{
						current_local_distance += link.length / 2.0f;
						current_local_distance += g_link_vector[g_number_of_links - 1].length / 2.0f;
					}
				}
				else
				{
					current_local_distance = link.length / 2.0f;
				}

				link.local_distance = current_local_distance;

				//long link_key = internal_from_node_seq_no * 100000 + internal_to_node_seq_no;

				//g_link_key_to_seq_no_map[link_key] = link.link_seq_no;

				g_link_vector.push_back(link);

				g_number_of_links++;

				if (g_number_of_links % 1000 == 0)
					cout << "reading " << g_number_of_links << " links.. " << endl;
			}
		}

		cout << "number of links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);

		parser_link.CloseCSVFile();

		// set NFD zone jam 

		g_number_of_agents = 0;


		CCSVParser parser_agent;

		if (parser_agent.OpenCSVFile("input_agent.csv", true))   // read agent as demand input 
		{
			while (parser_agent.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
			{
				CAgent agent;  // create an agent object 
				if (parser_agent.GetValueByFieldName("agent_id", agent.agent_id) == false)
					continue;

				if (g_number_of_agents >= g_max_number_of_agents)
					break;

				if (parser_agent.GetValueByFieldName("agent_id", agent.agent_id) == false)
					continue;

				int origin_node_id = 0;
				int destination_node_id = 0;
				parser_agent.GetValueByFieldName("from_origin_node_id", origin_node_id, false);

				parser_agent.GetValueByFieldName("to_destination_node_id", destination_node_id, false);

				if (g_internal_node_seq_no_map.find(origin_node_id) == g_internal_node_seq_no_map.end() || g_internal_node_seq_no_map.find(destination_node_id) == g_internal_node_seq_no_map.end())
					continue;  // skip this record, if origin node and destiantion node have not been defined.

				agent.origin_node_seq_no = g_internal_node_seq_no_map[origin_node_id];
				agent.destination_node_seq_no = g_internal_node_seq_no_map[destination_node_id];


				parser_agent.GetValueByFieldName("departure_time_in_min", agent.departure_time_in_min);

				parser_agent.GetValueByFieldName("PCE", agent.PCE_factor);


				agent.origin_node_id = origin_node_id;//luchao
				agent.destination_node_id = destination_node_id;//luchao

				g_agent_vector.push_back(agent);

				g_internal_agent_no_map[agent.agent_id] = g_number_of_agents;
				g_external_agent_id_map[g_number_of_agents] = agent.agent_id;
				//g_agent_map[agent.agent_id] = agent;//luchao

				//To Do 1: load agent path from field path_node_sequence
				// initial loading multiplier: 0.66666

				g_number_of_agents++;
				if (g_number_of_agents % 1000 == 0)
					cout << "reading = " << g_number_of_agents / 1000 << " k agents..." << endl;
			}
		}

		parser_agent.CloseCSVFile();


		cout << "number of agents = " << g_agent_vector.size() << endl;


		CCSVParser gps_parser;

		if (gps_parser.OpenCSVFile("input_GPS.csv", true))   // read agent as demand input 
		{
			while (gps_parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
			{
				int agent_id = 0;
				if (gps_parser.GetValueByFieldName("agent_id", agent_id) == false)//luchao
					continue;

				double x = 0;
				double y = 0;
				int t = 0;
				//int v = agent_id - 1;  // to do use map 
				gps_parser.GetValueByFieldName("time_interval_no", t, false);
				gps_parser.GetValueByFieldName("x", x, false);
				gps_parser.GetValueByFieldName("y", y, false);

				CGPSPoint GPSPoint;
				GPSPoint.x = x;
				GPSPoint.y = y;
				GPSPoint.time_interval_no = t;
				g_agent_vector[g_internal_agent_no_map[agent_id]].m_GPSPointVector.push_back(GPSPoint);

				g_agent_vector[g_internal_agent_no_map[agent_id]].time_stamp_gps_point_no_map[t] = g_agent_vector[g_internal_agent_no_map[agent_id]].m_GPSPointVector.size() - 1;

				/*
				if (g_agent_map[agent_id]->GPSPointVector.size() > 1)
				{
					GPSLink *tempGPSLink = new GPSLink();
					tempGPSLink->From_GPSPoint = g_agent_map[agent_id]->GPSPointVector[g_agent_map[agent_id]->GPSPointVector.size() - 2];
					tempGPSLink->To_GPSPoint = g_agent_map[agent_id]->GPSPointVector[g_agent_map[agent_id]->GPSPointVector.size() - 1];
					g_agent_map[agent_id]->GPSLinkVector.push_back(tempGPSLink);
				}//luchao}
				*/
			}
			gps_parser.CloseCSVFile();

		}

		// step 5: read the input partial schedule


		//g_WriteNetworkBinFile();
	}

	g_number_of_simulation_minutes = (g_Simulation_EndTimeInMin + g_Post_Simulation_DurationInMin);

	//To Do 2: load input_sensor_data // speed, link volume, link density, per k min: starting time, ending time, from node, to node
	//To Do 3: load input_senario  // for prediction outflow_capacity, predicted free-flow speed
//
}

class CNode2NodeAccessibility
{
public:
	int from_node_no;
	int to_node_no;
	float travel_cost;
};


class CSTS_State  //class for space time states
{
public:
	float m_speed;

	std::vector<int> m_outgoing_state_index_vector;
	std::vector<int> m_outgoing_state_transition_cost_vector;

};

std::vector<CSTS_State> g_STSStateVector;

void g_add_state_transition(int from_state, int to_state, float TransitionCost)
{
	g_STSStateVector[from_state].m_outgoing_state_index_vector.push_back(to_state);  // link my own state index to the parent state
	g_STSStateVector[from_state].m_outgoing_state_transition_cost_vector.push_back(TransitionCost);  // link my own state index to the parent state
}

class STSNetwork  // mainly for STS shortest path calculation
{
public:
	int m_threadNo;  // internal thread number 
	std::vector<int>  m_agent_vector; // assigned agents for computing 

	int m_number_of_nodes, m_number_of_time_intervals;

	int m_origin_node;
	int m_departure_time_beginning;
	int m_arrival_time_ending;

	float** m_label_cost;
	int** m_node_predecessor;
	int** m_time_predecessor;
	int** m_state_predecessor;

	STSNetwork()
	{
		m_origin_node = -1;
		m_label_cost = NULL;
		m_node_predecessor = NULL;
		m_time_predecessor = NULL;
		m_state_predecessor = NULL;
	}

	void AllocateSTSMemory(int number_of_nodes, int number_of_time_intervals)
	{
		m_number_of_nodes = number_of_nodes;
		m_number_of_time_intervals = number_of_time_intervals;

		m_label_cost = Allocate2DDynamicArray<float>(m_number_of_nodes, m_number_of_time_intervals);
		m_node_predecessor = Allocate2DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals);
		m_time_predecessor = Allocate2DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals);
		m_state_predecessor = Allocate2DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals);
	}

	~STSNetwork()
	{
		Deallocate2DDynamicArray<float>(m_label_cost, m_number_of_nodes);
		Deallocate2DDynamicArray<int>(m_node_predecessor, m_number_of_nodes);
		Deallocate2DDynamicArray<int>(m_time_predecessor, m_number_of_nodes);
		Deallocate2DDynamicArray<int>(m_state_predecessor, m_number_of_nodes);
	}

	//parallel computing version
	//float optimal_STS_dynamic_programming(int departure_time_beginning, int arrival_time_ending)
	float optimal_STS_dynamic_programming(int agent_no)
	{
		CAgent* p_agent = &(g_agent_vector[agent_no]); //the first agent is the current agent

		float total_cost = _MAX_LABEL_COST;

		m_departure_time_beginning = p_agent->m_GPSPointVector[0].time_interval_no;
		m_arrival_time_ending = min(p_agent->m_GPSPointVector.back().time_interval_no, g_number_of_simulation_intervals - 1);
		m_origin_node = g_internal_node_seq_no_map[p_agent->origin_node_id];

		for (int i = 0; i < m_number_of_nodes; i++)
		{
			// to do: only update node label on the agent path
			for (int t = m_departure_time_beginning; t <= m_arrival_time_ending; t++)
			{

				m_label_cost[i][t] = _MAX_LABEL_COST;
				m_node_predecessor[i][t] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
				m_time_predecessor[i][t] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
				//m_state_predecessor[i][t] = -1;

			}
		}

		//step 2: Initialization for origin node at the preferred departure time, at departure time

		m_label_cost[m_origin_node][m_departure_time_beginning] = 0;

		// step 3: //dynamic programming , absoluate time
		if (g_dp_algorithm_debug_flag == 1)
		{
			fprintf(g_pFileDebugLog, "****************starting of the DP****************\n");
			fprintf(g_pFileDebugLog, "agent_id = %d\n", p_agent->agent_id);
		}

		float link_cost_temp;
		float gps_x, gps_y, from_node_x, to_node_x, from_node_y, to_node_y;

		for (int t = m_departure_time_beginning; t <= m_arrival_time_ending; t++)  //first loop: time
		{
			//cout << t << endl;

			if (g_dp_algorithm_debug_flag == 1)
			{
				fprintf(g_pFileDebugLog, "t = %d\n", t);
			}

			for (int link = 0; link < g_link_vector.size(); link++)
			{
				if (m_label_cost[g_link_vector[link].from_node_seq_no][t] == _MAX_LABEL_COST)
				{
					continue;
				}

				from_node_x = g_node_vector[g_link_vector[link].from_node_seq_no].x;
				to_node_x = g_node_vector[g_link_vector[link].to_node_seq_no].x;
				from_node_y = g_node_vector[g_link_vector[link].from_node_seq_no].y;
				to_node_y = g_node_vector[g_link_vector[link].to_node_seq_no].y;

				for (int t1 = t + 1; t1 <= m_arrival_time_ending; t1++)
				{
					if (g_node_vector[g_link_vector[link].to_node_seq_no].node_state_matrix[t1] == -1)
					{
						continue;
					}

					link_cost_temp = 0;

					gps_x = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t]].x;
					gps_y = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t]].y;
					link_cost_temp += sqrt(pow(from_node_x - gps_x, 2) + pow(from_node_y - gps_y, 2));

					for (int tau = t + 1; tau < t1; tau++)
					{
						gps_x = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[tau]].x;
						gps_y = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[tau]].y;
						link_cost_temp += sqrt(pow((tau - t) / float(t1 - t) * (to_node_x - from_node_x) + from_node_x - gps_x, 2) + pow((tau - t) / float(t1 - t) * (to_node_y - from_node_y) + from_node_y - gps_y, 2));
					}

					gps_x = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t1]].x;
					gps_y = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t1]].y;
					link_cost_temp += sqrt(pow(to_node_x - gps_x, 2) + pow(to_node_y - gps_y, 2));

					if (m_label_cost[g_link_vector[link].from_node_seq_no][t] + link_cost_temp < m_label_cost[g_link_vector[link].to_node_seq_no][t1])
					{
						m_label_cost[g_link_vector[link].to_node_seq_no][t1] = m_label_cost[g_link_vector[link].from_node_seq_no][t] + link_cost_temp;
						m_node_predecessor[g_link_vector[link].to_node_seq_no][t1] = g_link_vector[link].from_node_seq_no;  // pointer to previous NODE INDEX from the current label at current node and time
						m_time_predecessor[g_link_vector[link].to_node_seq_no][t1] = t;
					}
				}
			}

			if (t < m_arrival_time_ending)
			{
				for (int node = 0; node < g_node_vector.size(); node++)
				{
					link_cost_temp = 0;
					from_node_x = g_node_vector[g_node_vector[node].node_seq_no].x;
					from_node_y = g_node_vector[g_node_vector[node].node_seq_no].y;
					gps_x = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t]].x;
					gps_y = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t]].y;

					link_cost_temp += sqrt(pow(from_node_x - gps_x, 2) + pow(from_node_y - gps_y, 2));

					gps_x = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t + 1]].x;
					gps_y = p_agent->m_GPSPointVector[p_agent->time_stamp_gps_point_no_map[t + 1]].y;
					link_cost_temp += sqrt(pow(from_node_x - gps_x, 2) + pow(from_node_y - gps_y, 2));

					if (m_label_cost[g_node_vector[node].node_seq_no][t] + link_cost_temp < m_label_cost[g_node_vector[node].node_seq_no][t + 1])
					{
						m_label_cost[g_node_vector[node].node_seq_no][t + 1] = m_label_cost[g_node_vector[node].node_seq_no][t] + link_cost_temp;
						m_node_predecessor[g_node_vector[node].node_seq_no][t + 1] = g_node_vector[node].node_seq_no;  // pointer to previous NODE INDEX from the current label at current node and time
						m_time_predecessor[g_node_vector[node].node_seq_no][t + 1] = t;
					}
				}
			}

		} // for all time t

		if (g_dp_algorithm_debug_flag == 1)
		{
			fprintf(g_pFileDebugLog, "****************End of the DP****************\n");
		}

		return total_cost;
	}

	void find_STS_path_for_agents_assigned_for_this_thread(int number_of_threads, int assignment_iteration_no)
	{


		// perform one to all STS shortest path
		//int return_value = optimal_STS_dynamic_programming(m_departure_time_beginning, m_arrival_time_ending);

		for (int i = 0; i < m_agent_vector.size(); i++)
		{
			int reversed_path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };
			int reversed_path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };
			int reversed_path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };
			float reversed_path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };

			int path_node_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };
			int path_link_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };
			int path_time_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };
			int path_state_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };
			float path_cost_sequence[_MAX_NUMBER_OF_TIME_INTERVALS] = { -1 };

			float return_value = optimal_STS_dynamic_programming(m_agent_vector[i]);


			CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

			p_agent->path_link_seq_no_vector.clear();  // reset;
			p_agent->path_timestamp_vector.clear();
			p_agent->path_node_id_vector.clear();  // reset;

			if (return_value == -1)
			{
				fprintf(g_pFileDebugLog, "agent %d with can not find destination node,\n", i);
				continue;
			}

			int current_node_seq_no;
			int current_link_seq_no;

			//step 4: back trace from the destination node to find the shortest path from shortest path tree 
			int destination_node_seq_no = g_internal_node_seq_no_map[p_agent->destination_node_id];

			float total_cost = _MAX_LABEL_COST;

			int min_cost_time_index = m_arrival_time_ending;

			total_cost = m_label_cost[destination_node_seq_no][min_cost_time_index];
			//cout << total_cost << endl;
			for (int t = m_arrival_time_ending; t > m_departure_time_beginning; t--)
			{
				if (m_label_cost[destination_node_seq_no][t] <= total_cost)
				{
					min_cost_time_index = t;
					total_cost = m_label_cost[destination_node_seq_no][t];
				}
			}

			//cout << total_cost << endl;

			// step 2: backtrack to the origin (based on node and time predecessors)
			int	node_size = 0;
			reversed_path_node_sequence[node_size] = destination_node_seq_no; //record the first node backward, destination node
			reversed_path_time_sequence[node_size] = min_cost_time_index;

			reversed_path_cost_sequence[node_size] = m_label_cost[destination_node_seq_no][min_cost_time_index];

			node_size++;

			int pred_node = m_node_predecessor[destination_node_seq_no][min_cost_time_index];
			int pred_time = m_time_predecessor[destination_node_seq_no][min_cost_time_index];

			while (pred_node != -1 && node_size < _MAX_NUMBER_OF_TIME_INTERVALS) // scan backward in the predessor array of the shortest path calculation results
			{
				reversed_path_node_sequence[node_size] = pred_node;
				reversed_path_time_sequence[node_size] = pred_time;
				reversed_path_cost_sequence[node_size] = m_label_cost[pred_node][pred_time];

				node_size++;

				//record current values of node and time predecessors, and update PredNode and PredTime

				int pred_node_record = pred_node;
				int pred_time_record = pred_time;

				pred_node = m_node_predecessor[pred_node_record][pred_time_record];
				pred_time = m_time_predecessor[pred_node_record][pred_time_record];
			}

			//reverse the node sequence 
			for (int n = 0; n < node_size; n++)
			{
				path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
				path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];
				path_cost_sequence[n] = reversed_path_cost_sequence[node_size - n - 1];
			}

			for (int i = 0; i < node_size; i++)  // for each node 
			{
				p_agent->path_node_id_vector.push_back(g_internal_node_seq_no_to_node_id_map[path_node_sequence[i]]);
				p_agent->path_timestamp_vector.push_back(path_time_sequence[i]);
				g_node_vector[path_node_sequence[i]].node_state_matrix[path_time_sequence[i]] = -1;
			}

			for (int i = 0; i < node_size - 1; i++)  // for each link, 
			{
				int link_no = g_GetLinkSeqNoByNodeSeq(g_internal_node_seq_no_map[p_agent->path_node_id_vector[i]], g_internal_node_seq_no_map[p_agent->path_node_id_vector[i + 1]]);
				path_link_sequence[i] = link_no;

				if (link_no == -1) // by zyx to skip the waiting arcs
				{
					continue;
				}

				p_agent->path_link_seq_no_vector.push_back(link_no);
			}

			float travel_time_return_value = path_time_sequence[node_size - 1] - path_time_sequence[0];

			int path_number_of_nodes = node_size;
		}
	}
};

int g_number_of_CPU_threads()
{
	int number_of_threads = omp_get_max_threads();
	/*
	int max_number_of_threads = 8;

	if (number_of_threads > max_number_of_threads)
		number_of_threads = max_number_of_threads;
	*/
	return number_of_threads;
}


STSNetwork* pSTSNetwork = NULL;
list<CAgentElement> g_agent_pointer_list;  // ready to active, and still in the network

void g_OutAgentCSVFile_FromSimulation()
{
	FILE* g_pFileAgent = NULL;
	g_pFileAgent = fopen("output_agent.csv", "w");

	if (g_pFileAgent == NULL)
	{
		cout << "File output_agent.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	else
	{
		fprintf(g_pFileAgent, "agent_id, from_origin_node_id, to_destination_node_id, path_node_id_sequence,path_timestamp_sequence");
		if (g_number_of_seconds_per_interval == 1)
			fprintf(g_pFileAgent, "path_time_sequence_in_sec,");

		fprintf(g_pFileAgent, "\n");

		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			fprintf(g_pFileAgent, "%d,%d,%d,",
				p_agent->agent_id,
				p_agent->origin_node_id,
				p_agent->destination_node_id
			);

			for (int i = 0; i < p_agent->path_node_id_vector.size(); i++)
			{
				fprintf(g_pFileAgent, "%d;", p_agent->path_node_id_vector[i]);
			}

			fprintf(g_pFileAgent, ",");

			for (int i = 0; i < p_agent->path_timestamp_vector.size(); i++)
			{
				fprintf(g_pFileAgent, "%d;", p_agent->path_timestamp_vector[i]);
			}

			fprintf(g_pFileAgent, "\n");
		}
		//
		fclose(g_pFileAgent);
	}
}


bool g_SequentialOptimization()
{
	CSTS_State element;

	g_STSStateVector.push_back(element);

	g_add_state_transition(0, 0, 0);

	int number_of_threads = g_number_of_CPU_threads();

	//number_of_threads = 1;

	pSTSNetwork = new STSNetwork[number_of_threads]; // create n copies of network, each for a subset of agents to use 

	for (int i = 0; i < number_of_threads; i++)
	{
		pSTSNetwork[i].AllocateSTSMemory(g_number_of_nodes, g_number_of_simulation_intervals);
	}

	//pSTSNetwork[0].AllocateSTSMemory(g_number_of_nodes, g_number_of_simulation_intervals);

	for (int l = 0; l < g_link_vector.size(); l++)
	{
		g_link_vector[l].Setup_State_Dependent_Data_Matrix();
	}


	for (int n = 0; n < g_node_vector.size(); n++)
	{
		g_node_vector[n].Setup_Node_State_Matrix();
	}

	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		int thread_no = a % number_of_threads;

		pSTSNetwork[thread_no].m_agent_vector.push_back(a);
	}

	int LR_iteration = 0;

	//#pragma omp parallel for
	for (int thread_no = 0; thread_no < number_of_threads; thread_no++)
	{
		pSTSNetwork[thread_no].find_STS_path_for_agents_assigned_for_this_thread(number_of_threads, LR_iteration);
	}

	g_OutAgentCSVFile_FromSimulation();

	cout << "End of Sequential Optimization Process. " << endl;

	return true;
}

int main(int argc, TCHAR* argv[], TCHAR* envp[])
{
	// definte timestamps
	clock_t start_t, end_t, total_t;

	start_t = clock();
	g_pFileDebugLog = fopen("Debug.txt", "w");

	if (g_pFileDebugLog == NULL)
	{
		cout << "File Debug.txt cannot be opened." << endl;
		g_ProgramStop();
	}
	g_pFileOutputLog = fopen("output_solution.csv", "w");
	if (g_pFileOutputLog == NULL)
	{
		cout << "File output_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	g_pNGSIMOuputLog = fopen("output_NGSIM.csv", "w");

	if (g_pNGSIMOuputLog == NULL)
	{
		cout << "File output_NGSIM.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	// step 1: read input data of network and demand agent
	g_ReadInputData();
	g_SequentialOptimization();

	end_t = clock();
	total_t = (end_t - start_t);
	cout << "CPU Running Time = " << total_t / 1000.0f << " seconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %.3f seconds\n", total_t / 1000.0);
	fprintf(g_pFileOutputLog, "CPU Running Time =,%.3f, seconds\n", total_t / 1000.0);

	fclose(g_pFileOutputLog);
	fclose(g_pFileDebugLog);

	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;
	cout << "done." << endl;

	g_node_vector.clear();
	g_link_vector.clear();

	getchar();

	return 1;
}