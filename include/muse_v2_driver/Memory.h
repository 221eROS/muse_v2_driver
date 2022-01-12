#ifndef MEMORY_H
#define MEMORY_H

#include <ros/package.h>

#include <muse_v2_driver/Muse.h>
#include <muse_v2_driver/MemoryManagement.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

#include <muse_v2_driver/RosLog.h>

#include <iostream>
#include <fstream>

#include <sys/stat.h>

namespace muse_v2_driver {

	class Memory
	{
	public:

		struct CommandList
		{

			bool get_available_memory = false;
			bool erase_memory = false;
			bool read_memory = false;
			int read_file = 0;
			bool get_files = false;


			bool operator==(const CommandList& rhs) const {
				return  (
					(get_available_memory == rhs.get_available_memory) &&
					(erase_memory == rhs.erase_memory) &&
					(read_memory == rhs.read_memory) &&
					(read_file == rhs.read_file) &&
					(get_files == rhs.get_files)
					);
			}

		} default_command_list;

		CommandList input_command, received_command;

		Memory() = default;
		~Memory() = default;

		void setupInputCommands(ros::NodeHandle& node);
		bool logger(MemoryManagement::Request& req, MemoryManagement::Response& res, Muse* muse);
	

	};
}

#endif 
