// MapMatching.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <list> 
#include <omp.h>
#include <algorithm>
#include <time.h>
#include "CSVParser.h"
#include <math.h>

using namespace std;

#define _MAX_LABEL_COST 999999

int g_max_number_of_threads = 4;
int g_dp_algorithm_debug_flag = 1;

int g_number_of_nodes = 0;
int g_number_of_links = 0;
int g_number_of_agents = 0;

std::map<int, int> g_internal_node_seq_no_map;
std::map<int, int> g_internal_node_seq_no_to_node_id_map;
std::map<int, int> g_internal_link_no_map;
std::map<int, int> g_external_link_id_map;
std::map<int, int> g_internal_agent_no_map;
std::map<int, int> g_external_agent_id_map;

FILE* g_pFileDebugLog = NULL;


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


class CNode
{
public:
	CNode()
	{
	}

	int node_seq_no;  // sequence number
	int node_id;      //external node number
	string name;

	double x;
	double y;

};


class CLink
{
public:
	CLink()
	{
	}

	~CLink()
	{

	}

	int link_id;
	string name;
	int from_node_id;
	int to_node_id;

	int link_seq_no;
	int from_node_seq_no;
	int to_node_seq_no;
};

std::vector<CNode> g_node_vector;
std::vector<CLink> g_link_vector;


class CGPSPoint
{
public:
	CGPSPoint() {}
	~CGPSPoint() {}

public:
	double x;
	double y;
	//double time_interval_no;
	string time_str;		// hhmm:ss
	int time_in_second;
};


class CAgent
{
public:
	CAgent()
	{
	}

	int agent_id;
	int origin_node_id;
	int destination_node_id;
	int origin_node_seq_no;
	int destination_node_seq_no;

	int start_time_in_second;
	int end_time_in_second;
	int duration_in_second;
	
	std::vector<int> path_node_seq_no_vector;
	std::vector<float> path_node_timestamp_vector;
	std::vector<int> path_link_seq_no_vector;

	std::vector<int> path_node_id_vector;
	std::vector<int> path_timestamp_vector;

	int number_of_gps_points;
	std::vector<CGPSPoint> m_GPSPointVector;
	vector<int> gps_point_index_vector;		// sort by time
	std::map<int, int> time_stamp_gps_point_no_map; // map time stamp to gps point no


};


vector<CAgent> g_agent_vector;

void g_ProgramStop()
{

	cout << "AgentLite Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};


int timestr2second(string time_str)
{
	string hh = time_str.substr(0, 2);
	string mm = time_str.substr(2, 2);
	string ss = time_str.substr(5, 2);
	int hhi = stoi(hh);
	int mmi = stoi(mm);
	int ssi = stoi(ss);
	return hhi * 3600 + mmi * 60 + ssi;
}


string second2timestr(int time_int)
{	
	int hhi = time_int / 3600;
	int mmi = (time_int - 3600 * hhi) / 60;
	int ssi = time_int - 3600 * hhi - 60 * mmi;
	string hh = hhi < 10 ? "0" + to_string(hhi) : to_string(hhi);
	string mm = mmi < 10 ? "0" + to_string(mmi) : to_string(mmi);
	string ss = ssi < 10 ? "0" + to_string(ssi) : to_string(ssi);
	return hh + mm + ":" + ss;
}


void g_ReadInputData()
{
	CCSVParser parser;
	if (parser.OpenCSVFile("node.csv", true))
	{
		int node_id;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;

			if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
			{
				cout << "warning: duplicate definition of node " << node_id << " was detected\n";
				continue;
			}

			CNode node;
			node.node_id = node_id;
			node.node_seq_no = g_number_of_nodes++;
			parser.GetValueByFieldName("x_coord", node.x, false);
			parser.GetValueByFieldName("y_coord", node.y, false);

			g_node_vector.push_back(node);
			g_internal_node_seq_no_map[node_id] = node.node_seq_no;
			g_internal_node_seq_no_to_node_id_map[node.node_seq_no] = node_id;
			if (g_number_of_nodes % 1000 == 0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}

		cout << "number of nodes = " << g_number_of_nodes << endl;
		parser.CloseCSVFile();
	}
	else
	{
		cout << "Cannot open file node.csv" << endl;
		g_ProgramStop();
	}


	CCSVParser parser_link;
	if (parser_link.OpenCSVFile("road_link.csv", true))
	{
		while (parser_link.ReadRecord())
		{
			CLink link;

			if (parser_link.GetValueByFieldName("link_id", link.link_id) == false)
				continue;
			if (parser_link.GetValueByFieldName("from_node_id", link.from_node_id) == false)
				continue;
			if (parser_link.GetValueByFieldName("to_node_id", link.to_node_id) == false)
				continue;

			if (g_internal_node_seq_no_map.find(link.from_node_id) == g_internal_node_seq_no_map.end())
			{
				cout << "warning: from_node_id " << link.from_node_id << " of link " << link.link_id << " has not been defined in node.csv\n";
				continue;
			}
			if (g_internal_node_seq_no_map.find(link.to_node_id) == g_internal_node_seq_no_map.end())
			{
				cout << "warning: to_node_id " << link.to_node_id << " of link " << link.link_id << " has not been defined in node.csv\n";
				continue;
			}

			link.from_node_seq_no = g_internal_node_seq_no_map[link.from_node_id];
			link.to_node_seq_no = g_internal_node_seq_no_map[link.to_node_id];
			link.link_seq_no = g_number_of_links++;

			g_internal_link_no_map[link.link_id] = link.link_seq_no;
			g_external_link_id_map[link.link_seq_no] = link.link_id;
			//g_node_vector[internal_from_node_seq_no].m_outgoing_node_vector.push_back(link);
			g_link_vector.push_back(link);

			if (g_number_of_links % 1000 == 0)
				cout << "reading " << g_number_of_links << " links.. " << endl;
		}

		cout << "number of links = " << g_number_of_links << endl;
		parser_link.CloseCSVFile();
	}
	else
	{
		cout << "Cannot open file road_link.csv" << endl;
		g_ProgramStop();
	}


	CCSVParser parser_agent;
	if (parser_agent.OpenCSVFile("input_agent.csv", true))
	{
		int origin_node_id, destination_node_id;

		while (parser_agent.ReadRecord())
		{
			CAgent agent;
			if (parser_agent.GetValueByFieldName("agent_id", agent.agent_id) == false)
				continue;

			parser_agent.GetValueByFieldName("from_origin_node_id", origin_node_id, false);
			parser_agent.GetValueByFieldName("to_destination_node_id", destination_node_id, false);

			if (g_internal_node_seq_no_map.find(origin_node_id) == g_internal_node_seq_no_map.end())
			{
				cout << "warning: from_origin_node_id " << origin_node_id << " of agent " << agent.agent_id << " has not been defined in node.csv\n";
				continue;
			}
			if (g_internal_node_seq_no_map.find(destination_node_id) == g_internal_node_seq_no_map.end())
			{
				cout << "warning: destination_node_id " << destination_node_id << " of agent " << agent.agent_id << " has not been defined in node.csv\n";
				continue;
			}

			agent.origin_node_id = origin_node_id;
			agent.destination_node_id = destination_node_id;
			agent.origin_node_seq_no = g_internal_node_seq_no_map[origin_node_id];
			agent.destination_node_seq_no = g_internal_node_seq_no_map[destination_node_id];

			g_agent_vector.push_back(agent);
			g_internal_agent_no_map[agent.agent_id] = g_number_of_agents;
			g_external_agent_id_map[g_number_of_agents] = agent.agent_id;

			g_number_of_agents++;
			if (g_number_of_agents % 1000 == 0)
				cout << "reading = " << g_number_of_agents / 1000 << " k agents..." << endl;
		}

		parser_agent.CloseCSVFile();
		cout << "number of agents = " << g_agent_vector.size() << endl;
	}
	else
	{
		cout << "Cannot open file input_agent.csv" << endl;
		g_ProgramStop();
	}


	CCSVParser gps_parser;
	if (gps_parser.OpenCSVFile("trajectory.csv", true))
	{
		double x, y;
		string time_stamp;
		int time_in_second;
		while (gps_parser.ReadRecord())
		{
			int agent_id = 0;
			if (gps_parser.GetValueByFieldName("agent_id", agent_id) == false)
				continue;

			gps_parser.GetValueByFieldName("x_coord", x, false);
			gps_parser.GetValueByFieldName("y_coord", y, false);
			gps_parser.GetValueByFieldName("timestamp", time_stamp);
			time_in_second = timestr2second(time_stamp);
			
			CGPSPoint GPSPoint;
			GPSPoint.x = x;
			GPSPoint.y = y;
			GPSPoint.time_str = time_stamp;
			GPSPoint.time_in_second = time_in_second;
			g_agent_vector[g_internal_agent_no_map[agent_id]].m_GPSPointVector.push_back(GPSPoint);
			g_agent_vector[g_internal_agent_no_map[agent_id]].time_stamp_gps_point_no_map[time_in_second] =
				g_agent_vector[g_internal_agent_no_map[agent_id]].m_GPSPointVector.size() - 1;
		}
		gps_parser.CloseCSVFile();

		// sort gps points for each agent
		CAgent *p_agent;
		for (int i = 0; i < g_number_of_agents; i++)
		{
			p_agent = &g_agent_vector[i];
			p_agent->number_of_gps_points = p_agent->m_GPSPointVector.size();

			for (int k = 0; k < p_agent->number_of_gps_points; k++)
				p_agent->gps_point_index_vector.push_back(k);
			sort(p_agent->gps_point_index_vector.begin(), p_agent->gps_point_index_vector.end(), [&](const int& a, const int& b) {
				return (p_agent->m_GPSPointVector[a].time_in_second < p_agent->m_GPSPointVector[b].time_in_second); });
			p_agent->start_time_in_second = p_agent->m_GPSPointVector[p_agent->gps_point_index_vector[0]].time_in_second;
			p_agent->end_time_in_second = p_agent->m_GPSPointVector[p_agent->gps_point_index_vector.back()].time_in_second;
			p_agent->duration_in_second = p_agent->end_time_in_second - p_agent->start_time_in_second + 1;
		}
	}
	else
	{
		cout << "Cannot open file trajectory.csv" << endl;
		g_ProgramStop();
	}
}


class STSNetwork  // mainly for STS shortest path calculation
{
public:
	STSNetwork()
	{
		m_origin_node = -1;
		m_label_cost = NULL;
		m_node_predecessor = NULL;
		m_time_predecessor = NULL;
		m_state_predecessor = NULL;
		m_max_duration_in_second = 0;
	}

	~STSNetwork()
	{
		Deallocate2DDynamicArray<float>(m_label_cost, m_number_of_nodes);
		Deallocate2DDynamicArray<int>(m_node_predecessor, m_number_of_nodes);
		Deallocate2DDynamicArray<int>(m_time_predecessor, m_number_of_nodes);
		Deallocate2DDynamicArray<int>(m_state_predecessor, m_number_of_nodes);
	}

	int m_threadNo;
	std::vector<int>  m_agent_vector;

	int m_max_duration_in_second;
	int m_number_of_nodes, m_number_of_time_intervals;

	int m_origin_node;
	int m_departure_time_beginning;
	int m_arrival_time_ending;

	float** m_label_cost;
	int** m_node_predecessor;
	int** m_time_predecessor;
	int** m_state_predecessor;

	void AllocateSTSMemory()
	{
		m_number_of_nodes = g_number_of_nodes;
		m_number_of_time_intervals = m_max_duration_in_second;

		m_label_cost = Allocate2DDynamicArray<float>(m_number_of_nodes, m_number_of_time_intervals);
		m_node_predecessor = Allocate2DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals);
		m_time_predecessor = Allocate2DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals);
		m_state_predecessor = Allocate2DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals);
	}

	//parallel computing version
	int optimal_STS_dynamic_programming(int agent_no)
	{
		CAgent* p_agent = &(g_agent_vector[agent_no]); //the first agent is the current agent

		m_departure_time_beginning = p_agent->start_time_in_second;
		m_arrival_time_ending = p_agent->end_time_in_second;
		m_origin_node = g_internal_node_seq_no_map[p_agent->origin_node_id];

		int number_of_time_stamps = p_agent->duration_in_second;

		for (int i = 0; i < m_number_of_nodes; i++)
		{
			// to do: only update node label on the agent path
			for (int tr = 0; tr < number_of_time_stamps; tr++)
			{
				m_label_cost[i][tr] = _MAX_LABEL_COST;
				m_node_predecessor[i][tr] = -1;
				m_time_predecessor[i][tr] = -1;
			}
		}

		//step 2: Initialization for origin node at the preferred departure time, at departure time
		m_label_cost[m_origin_node][0] = 0;

		// step 3: //dynamic programming , relative time
		if (g_dp_algorithm_debug_flag == 1)
		{
			fprintf(g_pFileDebugLog, "****************starting of the DP****************\n");
			fprintf(g_pFileDebugLog, "agent_id = %d\n", p_agent->agent_id);
		}

		float link_cost_temp;
		float gps_x, gps_y, from_node_x, to_node_x, from_node_y, to_node_y;

		int t, t1;
		for (int tr = 0; tr < number_of_time_stamps; tr++)
		{
			t = tr + m_departure_time_beginning;

			if (g_dp_algorithm_debug_flag == 1)
			{
				fprintf(g_pFileDebugLog, "t = %d\n", t);
			}

			for (int link = 0; link < g_link_vector.size(); link++)
			{
				if (m_label_cost[g_link_vector[link].from_node_seq_no][tr] == _MAX_LABEL_COST)
					continue;

				from_node_x = g_node_vector[g_link_vector[link].from_node_seq_no].x;
				to_node_x = g_node_vector[g_link_vector[link].to_node_seq_no].x;
				from_node_y = g_node_vector[g_link_vector[link].from_node_seq_no].y;
				to_node_y = g_node_vector[g_link_vector[link].to_node_seq_no].y;

				for (int tr1 = tr + 1; tr1 < number_of_time_stamps; tr1++)
				{
					t1 = tr1 + m_departure_time_beginning;
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

					if (m_label_cost[g_link_vector[link].from_node_seq_no][tr] + link_cost_temp < m_label_cost[g_link_vector[link].to_node_seq_no][tr1])
					{
						m_label_cost[g_link_vector[link].to_node_seq_no][tr1] = m_label_cost[g_link_vector[link].from_node_seq_no][tr] + link_cost_temp;
						m_node_predecessor[g_link_vector[link].to_node_seq_no][tr1] = g_link_vector[link].from_node_seq_no;  // pointer to previous NODE INDEX from the current label at current node and time
						m_time_predecessor[g_link_vector[link].to_node_seq_no][tr1] = tr;
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

					if (m_label_cost[g_node_vector[node].node_seq_no][tr] + link_cost_temp < m_label_cost[g_node_vector[node].node_seq_no][tr + 1])
					{
						m_label_cost[g_node_vector[node].node_seq_no][tr + 1] = m_label_cost[g_node_vector[node].node_seq_no][tr] + link_cost_temp;
						m_node_predecessor[g_node_vector[node].node_seq_no][tr + 1] = g_node_vector[node].node_seq_no;  // pointer to previous NODE INDEX from the current label at current node and time
						m_time_predecessor[g_node_vector[node].node_seq_no][tr + 1] = tr;
					}
				}
			}

		}

		if (g_dp_algorithm_debug_flag == 1)
		{
			fprintf(g_pFileDebugLog, "****************End of the DP****************\n");
		}

		return 1;
	}


	void find_STS_path_for_agents_assigned_for_this_thread()
	{
		int number_of_time_stampes;
		CAgent* p_agent;
		int *reversed_path_node_sequence, *reversed_path_time_sequence, *reversed_path_state_sequence;
		float *reversed_path_cost_sequence;
		int *path_node_sequence, *path_time_sequence, *path_state_sequence;
		float *path_cost_sequence;
		int return_value;

		for (int i = 0; i < m_agent_vector.size(); i++)
		{
			CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);
			number_of_time_stampes = p_agent->duration_in_second;

			reversed_path_node_sequence = new int[number_of_time_stampes];
			reversed_path_time_sequence = new int[number_of_time_stampes];
			reversed_path_state_sequence = new int[number_of_time_stampes];
			reversed_path_cost_sequence = new float[number_of_time_stampes];
			path_node_sequence = new int[number_of_time_stampes];
			path_time_sequence = new int[number_of_time_stampes];
			path_state_sequence = new int[number_of_time_stampes];
			path_cost_sequence = new float[number_of_time_stampes];

			return_value = optimal_STS_dynamic_programming(m_agent_vector[i]);
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
			int min_cost_time_index_r = number_of_time_stampes - 1;
			total_cost = m_label_cost[destination_node_seq_no][min_cost_time_index_r];

			for (int tr = number_of_time_stampes - 2; tr >= 0; tr--)
			{
				if (m_label_cost[destination_node_seq_no][tr] <= total_cost)
				{
					min_cost_time_index_r = tr;
					total_cost = m_label_cost[destination_node_seq_no][tr];
				}
			}

			// step 2: backtrack to the origin (based on node and time predecessors)
			int	node_size = 0;
			reversed_path_node_sequence[node_size] = destination_node_seq_no; //record the first node backward, destination node
			reversed_path_time_sequence[node_size] = min_cost_time_index_r + m_departure_time_beginning;
			reversed_path_cost_sequence[node_size] = m_label_cost[destination_node_seq_no][min_cost_time_index_r];
			node_size++;

			int pred_node = m_node_predecessor[destination_node_seq_no][min_cost_time_index_r];
			int pred_time_r = m_time_predecessor[destination_node_seq_no][min_cost_time_index_r];

			int pred_node_record, pred_time_record_r;

			while (pred_node != -1) // scan backward in the predessor array of the shortest path calculation results
			{
				reversed_path_node_sequence[node_size] = pred_node;
				reversed_path_time_sequence[node_size] = pred_time_r + m_departure_time_beginning;
				reversed_path_cost_sequence[node_size] = m_label_cost[pred_node][pred_time_r];

				node_size++;

				//record current values of node and time predecessors, and update PredNode and PredTime
				pred_node_record = pred_node;
				pred_time_record_r = pred_time_r;

				pred_node = m_node_predecessor[pred_node_record][pred_time_record_r];
				pred_time_r = m_time_predecessor[pred_node_record][pred_time_record_r];
			}

			//reverse the node sequence 
			for (int n = 0; n < node_size; n++)
			{
				path_node_sequence[n] = reversed_path_node_sequence[node_size - n - 1];
				path_time_sequence[n] = reversed_path_time_sequence[node_size - n - 1];
				path_cost_sequence[n] = reversed_path_cost_sequence[node_size - n - 1];
			}

			for (int i = 0; i < node_size; i++) 
			{
				p_agent->path_node_id_vector.push_back(g_internal_node_seq_no_to_node_id_map[path_node_sequence[i]]);
				p_agent->path_timestamp_vector.push_back(path_time_sequence[i]);
			}

			delete[] reversed_path_node_sequence, reversed_path_time_sequence, reversed_path_state_sequence, reversed_path_cost_sequence;
			delete[] path_node_sequence, path_time_sequence, path_state_sequence, path_cost_sequence;
		}
	}
};


int g_number_of_CPU_threads()
{
	int number_of_threads = omp_get_max_threads();
	if (number_of_threads <= g_max_number_of_threads)
		return number_of_threads;
	else
		return g_max_number_of_threads;
}


STSNetwork* pSTSNetworks = NULL;


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
		fprintf(g_pFileAgent, "agent_id, from_origin_node_id, to_destination_node_id, path_node_id_sequence,path_timestamp_sequence\n");

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
				fprintf(g_pFileAgent, "%s;", second2timestr(p_agent->path_timestamp_vector[i]));
			}

			fprintf(g_pFileAgent, "\n");
		}

		fclose(g_pFileAgent);
	}
}


bool g_SequentialOptimization()
{
	int number_of_threads = g_number_of_CPU_threads();
	pSTSNetworks = new STSNetwork[number_of_threads]; // create n copies of network, each for a subset of agents to use 

	STSNetwork *p_STSNetwork;
	for (int a = 0; a < g_agent_vector.size(); a++)
	{
		p_STSNetwork = &pSTSNetworks[a % number_of_threads];
		p_STSNetwork->m_agent_vector.push_back(a);
		if (g_agent_vector[a].duration_in_second > p_STSNetwork->m_max_duration_in_second)
			p_STSNetwork->m_max_duration_in_second = g_agent_vector[a].duration_in_second;
	}

	for (int i = 0; i < number_of_threads; i++)
	{
		pSTSNetworks[i].AllocateSTSMemory();
	}
	   	  
	#pragma omp parallel for
	for (int thread_no = 0; thread_no < number_of_threads; thread_no++)
	{
		pSTSNetworks[thread_no].find_STS_path_for_agents_assigned_for_this_thread();
	}

	g_OutAgentCSVFile_FromSimulation();

	cout << "End of Sequential Optimization Process. " << endl;

	return true;
}


int main(int argc, TCHAR* argv[], TCHAR* envp[])
{
	clock_t start_t, end_t, total_t;

	start_t = clock();
	g_pFileDebugLog = fopen("Debug.txt", "w");

	if (g_pFileDebugLog == NULL)
	{
		cout << "File Debug.txt cannot be opened." << endl;
		g_ProgramStop();
	}

	g_ReadInputData();
	g_SequentialOptimization();

	end_t = clock();
	total_t = (end_t - start_t);
	cout << "CPU Running Time = " << total_t / 1000.0f << " seconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %.3f seconds\n", total_t / 1000.0);
	fclose(g_pFileDebugLog);

	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;
	cout << "done." << endl;

	g_node_vector.clear();
	g_link_vector.clear();

	getchar();

	return 1;
}