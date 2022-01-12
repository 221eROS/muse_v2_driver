#include <muse_v2_driver/Memory.h>

void muse_v2_driver::Memory::setupInputCommands(ros::NodeHandle& node) {

	node.getParam("get_available_memory", input_command.get_available_memory);
	node.getParam("erase_memory", input_command.erase_memory);
	node.getParam("read_memory", input_command.read_memory);
	node.getParam("read_file", input_command.read_file);
	node.getParam("get_files", input_command.get_files);

}

bool muse_v2_driver::Memory::logger(MemoryManagement::Request& req, MemoryManagement::Response& res, Muse* muse) {

	bool out = false;

	ROS_INFO("request: get_available_memory=%s, erase_memory=%s, read_memory=%s, read_file=%d, get_files=%s",
		req.get_available_memory ? "true" : "false",
		req.erase_memory ? "true" : "false",
		req.read_memory ? "true" : "false",
		req.read_file,
		req.get_files ? "true" : "false"
	);

	if (req.get_available_memory) {
		if (!received_command.get_available_memory) {
			received_command.get_available_memory = true;
			ROS_INFO("Trying getting Available Memory.");
		}
		if (muse->serial->getAvailableMemory() > 0) {
			res.memory = muse->serial->getAvailableMemory();
			out = true;
			ROS_INFO("Sending back Available Memory.");
		}
	}

	if (req.erase_memory) {
		if (!received_command.erase_memory) {
			received_command.erase_memory = true;
			ROS_INFO("Trying erasing memory.");
		}
		if (muse->serial->eraseMemory()) {
			res.memory_erased = true;
			out = true;
			ROS_INFO("Memory erased.");
		}
	}

	if (req.get_files) {
		if (!received_command.get_files) {
			received_command.get_files = true;
			ROS_INFO("Trying getting file names list.");
		}
		if (!muse->serial->getFiles().empty()) {
			vector<pair<int, string>> file_names = muse->serial->getFiles();
			out = true;
			for (const auto& file : file_names)
				res.file_list.push_back(file.second);
			ROS_INFO("Sending back File names.");
		}

	}

	if (req.read_file != 0) {
		if (received_command.read_file == 0) {
			received_command.read_file = req.read_file;
			ROS_INFO("Trying reading file.");
		}

		out = true;

		if (muse->serial->readFile(req.read_file).empty()) {
			ROS_INFO("File not found. Please check the list of available files.");
			res.file_stored = false;
			res.dest_path = "";
			return out;
		}
		else {
			if (!muse->serial->getFiles().empty()) {
				vector<pair<int, string>> file_names = muse->serial->getFiles();
				vector<Log> logs = muse->serial->readFile(req.read_file);
				string dest_file = ros::package::getPath("muse_ros") + "/config/" + file_names.at(req.read_file - 1).second + ".txt";
				if (muse->serial->storeLogs(dest_file, logs)) {
					res.file_stored = true;
					res.dest_path = dest_file;
					ROS_INFO("Data stored at %s", dest_file.c_str());
				}

			}
		}

	}

	if (req.read_memory) {
		if (received_command.read_memory == false) {
			received_command.read_memory = true;
			ROS_INFO("Trying reading memory.");
		}

		int i = 0;
		int file_stored = 0;
		string dest_path = ros::package::getPath("muse_ros") + "/config/";
		out = true;
		
		if (muse->serial->getFiles().empty()) {
			ROS_INFO("Files not found");
			res.number_of_files_stored = 0;
			res.dest_path = "";
			return out;
		}

		vector<pair<int, string>> file_names = muse->serial->getFiles();

		while (i < file_names.size()) {
			ROS_INFO_STREAM("Processing file " << i + 1 << " of " << file_names.size() << ": " << file_names.at(i).second << ".txt");

			if (muse->serial->readFile(req.read_file).empty()) {
				ROS_INFO("File %d not properly read", i + 1);
				res.corrupt_stored_list.push_back(i);
			}

			std::vector<Log> logs = muse->serial->readFile(req.read_file);
			ROS_INFO("# of logs to store: %lu", logs.size());

			std::string dest_file = dest_path + file_names.at(i).second + ".txt";
			if (muse->serial->storeLogs(dest_file, logs)) {
				file_stored++;
				ROS_INFO("Data stored at %s", dest_file.c_str());
			}

			i++;
		}

		out = true;
		res.number_of_files_stored = file_stored;
		res.dest_path = dest_path;
	}

	return out;

}





