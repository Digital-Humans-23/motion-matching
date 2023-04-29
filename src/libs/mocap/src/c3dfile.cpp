#include "mocap/c3d/c3dfile.h"

#include <fstream>

#include "mocap/c3d/c3dutils.h"

using namespace std;

namespace crl::mocap {

bool C3DFile::load(const char *filename) {
    ifstream c3dstream(filename, ios::binary);
    if (!c3dstream) {
        cerr << "Could not open file " << filename << endl;
        return false;
    }

    // header section
    c3dstream.read(reinterpret_cast<char *>(&header), sizeof(C3DHeader));
    if (header.c3d_id != 0x50) {
        cerr << "Invalid C3D file!" << endl;
        return false;
    }

    // parameter Section
    readParameterSection(c3dstream);

    // we read out all the indices of the labels for easier future reference
    fillPointLabelMap();

    // 3d point data section (we currently ignore the analog part)
    readPointSection(c3dstream);

    c3dstream.close();

    return true;
}

FloatMarkerData C3DFile::getMarkerTrajectories(const char *point_name_str) {
    std::string marker_name(point_name_str);

    marker_name = marker_name.substr(0, marker_name.find_last_not_of(" ") + 1);

    if (label_point_map.find(marker_name) == label_point_map.end()) {
        cerr << "Error: could not find marker with name '" << marker_name << "'!" << endl;
        abort();
    }

    Sint16 index = label_point_map.find(marker_name)->second;

    assert(index >= 0 && index < 255);

    return float_point_data[index];
}

size_t C3DFile::getEventCount() {
    ParameterInfo event_used = getParamInfo("EVENT:USED");
    assert(event_used.data_type == 2);

    return static_cast<size_t>(event_used.int_data[0]);
}

EventInfo C3DFile::getEventInfo(size_t index) {
    assert(getEventCount() > index);

    EventInfo result;

    ParameterInfo contexts_info = getParamInfo("EVENT:CONTEXTS");
    DataMatrix<char> contexts_data(static_cast<unsigned int>(contexts_info.dimensions[1]), static_cast<unsigned int>(contexts_info.dimensions[0]));

    contexts_data.CopyFrom(contexts_info.char_data);
    string context = "";
    for (size_t j = 0; j < contexts_data.GetColumns(); j++) {
        context += contexts_data(index, j);
    }
    result.context = context.substr(0, context.find_last_not_of(' ') + 1);

    ParameterInfo labels_info = getParamInfo("EVENT:LABELS");
    DataMatrix<char> labels_data(static_cast<unsigned int>(labels_info.dimensions[1]), static_cast<unsigned int>(labels_info.dimensions[0]));

    labels_data.CopyFrom(labels_info.char_data);
    string label = "";
    for (size_t j = 0; j < labels_data.GetColumns(); j++) {
        label += labels_data(index, j);
    }
    result.label = label.substr(0, label.find_last_not_of(' ') + 1);

    ParameterInfo times_info = getParamInfo("EVENT:TIMES");
    DataMatrix<float> times_data(static_cast<unsigned int>(times_info.dimensions[1]), static_cast<unsigned int>(times_info.dimensions[0]));

    times_data.CopyFrom(times_info.float_data);
    result.time_minutes = times_data(index, 0);
    result.time_seconds = times_data(index, 1);

    return result;
}

std::string C3DFile::getParamString(const char *id_str) {
    ParameterInfo param_info;

    param_info = getParamInfo(id_str);

    assert(param_info.n_dimensions == 1);
    assert(check_param_type<char>(param_info.data_type));

    std::string result;

    int i;
    for (i = 0; i < param_info.dimensions[0]; i++)
        result += param_info.char_data[i];

    return result;
}

Sint8 C3DFile::getParamSint8(const char *id_str) {
    return getParamGeneric<Sint8>(id_str, static_cast<Sint8>(-1));
}

Sint16 C3DFile::getParamSint16(const char *id_str) {
    return getParamGeneric<Sint16>(id_str, static_cast<Sint16>(-1));
}

float C3DFile::getParamFloat(const char *id_str) {
    return getParamGeneric<float>(id_str, -1.f);
}

void C3DFile::readParameterSection(ifstream &datastream) {
    // locate the start of the parameter section
    int parameter_start = (header.first_parameter - 1) * 512;
    assert(parameter_start >= 512);
    datastream.seekg(parameter_start);

    // first there is a small parameter header that we want to read
    ParameterHeader pheader;
    datastream.read(reinterpret_cast<char *>(&pheader), sizeof(ParameterHeader));
    //cout << (int)pheader.reserved_1 << "\t" << (int)pheader.reserved_2 << "\t" << (int)pheader.num_parameter_blocks << "\t" << (int)pheader.processor_type << endl;

    // then all parameters and groups follow
    Sint8 peek_buffer[2];

    int param_count = 0;
    int group_count = 0;

    bool read_param = true;
    while (read_param) {
        datastream.read(reinterpret_cast<char *>(peek_buffer), sizeof(Sint8) * 2);
        datastream.seekg(-2, ios::cur);

        if (peek_buffer[1] < 0) {
            GroupInfo group_info = readGroupInfo(datastream);
            //print_group_info(group_info);

            group_infos.push_back(group_info);

            // we insert the group id into the map so that we can easier get to the
            // group data once we have the group id.
            // Note: we have to insert the negative value as the group id is always
            // negative!
            group_id_to_index_map[-group_info.id] = group_infos.size() - 1;

            // cout << "inserting group " << group_info.name << " at " << static_cast<int>(group_info.id) << endl;

            group_count++;
        } else {
            ParameterInfo param_info = readParameterInfo(datastream);
            //print_parameter_info(param_info);

            param_infos.push_back(param_info);
            param_count++;
            if (param_info.next_offset == 0)
                read_param = false;
        }
    }
    /*
	cout << "current position = " << datastream.tellg() << endl;
	cout << "Read " << param_count << " parameters" << endl;
	cout << "Read " << group_count << " groups" << endl;
	*/
}

GroupInfo C3DFile::readGroupInfo(ifstream &datastream) {
    GroupInfo result;

    // read name_length and id
    datastream.read(reinterpret_cast<char *>(&result), sizeof(Sint8) * 2);

    assert(result.name == NULL);
    if (result.name_length < 0) {
        result.name_length *= -1;
    }
    if (result.name_length > 127) {
        cerr << "Invalid group name length in C3D file!" << endl;
        abort();
    }
    result.name = new char[result.name_length + 1];
    datastream.read(result.name, result.name_length);
    result.name[result.name_length] = 0;

    datastream.read(reinterpret_cast<char *>(&result.next_offset), sizeof(Sint16));
    datastream.read(reinterpret_cast<char *>(&result.descr_length), sizeof(Sint8));

    assert(result.description == NULL);
    result.description = new char[result.descr_length + 1];
    datastream.read(result.description, result.descr_length);
    result.description[result.descr_length] = 0;

    return result;
}

ParameterInfo C3DFile::readParameterInfo(ifstream &datastream) {
    ParameterInfo result;
    datastream.read(reinterpret_cast<char *>(&result), sizeof(Sint8) * 2);

    assert(result.name == NULL);
    if (result.name_length < 0) {
        result.name_length *= -1;
        result.locked = true;
    } else {
        result.locked = false;
    }
    result.name = new char[result.name_length + 1];
    datastream.read(result.name, result.name_length);
    result.name[result.name_length] = 0;

    datastream.read(reinterpret_cast<char *>(&result.next_offset), sizeof(Sint16));
    datastream.read(reinterpret_cast<char *>(&result.data_type), sizeof(Uint8));
    datastream.read(reinterpret_cast<char *>(&result.n_dimensions), sizeof(Uint8));

    if (result.n_dimensions > 1) {
        // maximum of 7 dimensions (see c3d UG p. 50)
        assert(result.n_dimensions <= 7);
    }

    result.dimensions = new Uint8[result.n_dimensions];

    int i;

    if (result.n_dimensions == 0) {
        // if dim == 0 the data comes right away
        if (result.data_type == 1 || result.data_type == -1) {
            result.char_data = new char[1];
            datastream.read(reinterpret_cast<char *>(&result.char_data[0]), sizeof(char));
        } else if (result.data_type == 2) {
            result.int_data = new Sint16[1];
            datastream.read(reinterpret_cast<char *>(&result.int_data[0]), sizeof(Sint16));
        } else if (result.data_type == 4) {
            result.float_data = new float[1];
            datastream.read(reinterpret_cast<char *>(&result.float_data[0]), sizeof(float));
        }
    } else {
        // we first have to read out the dimensions of the values and then we can
        // store the values
        int count = 1;
        for (i = 0; i < result.n_dimensions; i++) {
            datastream.read(reinterpret_cast<char *>(&result.dimensions[i]), sizeof(Uint8));
            count *= result.dimensions[i];
        }

        if (result.data_type == 1 || result.data_type == -1) {
            result.char_data = new char[count];
            datastream.read(reinterpret_cast<char *>(&result.char_data[0]), sizeof(char) * count);
        } else if (result.data_type == 2) {
            result.int_data = new Sint16[count];
            datastream.read(reinterpret_cast<char *>(&result.int_data[0]), sizeof(Sint16) * count);
        } else if (result.data_type == 4) {
            result.float_data = new float[count];
            datastream.read(reinterpret_cast<char *>(&result.float_data[0]), sizeof(float) * count);
        }
    }

    datastream.read(reinterpret_cast<char *>(&result.descr_length), sizeof(Uint8));

    assert(result.description == NULL);
    result.description = new char[result.descr_length + 1];
    datastream.read(result.description, result.descr_length);
    result.description[result.descr_length] = 0;

    return result;
}

void C3DFile::readPointSection(ifstream &datastream) {
    float point_scale = getParamFloat("POINT:SCALE");

    //Check point scale value to figure out if the data is stored in integer or floating point format
    //if point_scale is negative the floating point format is used, other wise the integer format
    if (point_scale < 0.) {
        uses_integer_data = false;
        point_scale *= -1.;
    } else {
        uses_integer_data = true;
    }

    // compute the start of the point data
    int data_start = getParamSint16("POINT:DATA_START");
    data_start = 512 * (data_start - 1);

    assert(data_start > 512);

    datastream.seekg(data_start, ios::beg);

    Sint16 point_count = getParamSint16("POINT:USED");

    Uint16 frame_count = header.last_frame - header.first_frame + 1;

    /*
	float video_frame_rate = header.video_sampling_rate;
	float analog_frame_rate = header.video_sampling_rate * header.analog_channels_samples;
	Uint16 analog_channel_count = header.analog_channels;

	cout << endl;
	cout << "Frame Count       = " << frame_count << endl;
	cout << "Video Frame Rate  = " << video_frame_rate << endl;
	cout << "Analog Frame Rate = " << analog_frame_rate << endl;
	cout << "Analog channels   = " << analog_channel_count / header.analog_channels_samples << endl;
	cout << endl;
	*/

    // Populate point_data
    int i;
    for (i = 0; i < point_count; i++) {
        FloatMarkerData marker_data;
        float_point_data.push_back(marker_data);
        std::vector<FramePointInfo> frame_data;
        point_data.push_back(frame_data);
    }

    unsigned int frame_index;

    for (frame_index = 0; frame_index < frame_count; frame_index++) {
        // Read the frames
        for (i = 0; i < point_count; i++) {
            //read data from float data section
            if (!uses_integer_data) {
                FramePointInfo point_info;
                datastream.read(reinterpret_cast<char *>(&point_info), sizeof(FramePointInfo));

                float_point_data[i].x.push_back(point_info.x);
                float_point_data[i].y.push_back(point_info.y);
                float_point_data[i].z.push_back(point_info.z);

                float_point_data[i].cameras.push_back(point_info.cameras);
                float_point_data[i].residual.push_back(point_info.residual);

                point_data.at(i).push_back(point_info);
                //read data from int data section
            } else {
                FramePointIntInfo int_point_info;
                datastream.read(reinterpret_cast<char *>(&int_point_info), sizeof(FramePointIntInfo));

                FramePointInfo point_info;
                point_info.x = point_scale * int_point_info.x;
                point_info.y = point_scale * int_point_info.y;
                point_info.z = point_scale * int_point_info.z;
                point_info.cameras = int_point_info.cameras;
                point_info.residual = int_point_info.residual;

                float_point_data[i].x.push_back(point_info.x);
                float_point_data[i].y.push_back(point_info.y);
                float_point_data[i].z.push_back(point_info.z);

                float_point_data[i].cameras.push_back(point_info.cameras);
                float_point_data[i].residual.push_back(point_info.residual);

                point_data.at(i).push_back(point_info);
            }
        }

        datastream.seekg((header.analog_channels) * sizeof(float), ios::cur);
    }
}

void C3DFile::fillPointLabelMap() {
    ParameterInfo param_info;
    param_info = getParamInfo("POINT:LABELS");

    assert(param_info.data_type == -1);
    assert(param_info.n_dimensions == 2);

    // Warning: as column major is used, we have to switch row and column numbers!
    DataMatrix<char> data_matrix(static_cast<unsigned int>(param_info.dimensions[1]), static_cast<unsigned int>(param_info.dimensions[0]));

    data_matrix.CopyFrom(param_info.char_data);

    point_label.clear();

    size_t i, j;
    for (i = 0; i < data_matrix.GetRows(); i++) {
        std::string row;
        for (j = 0; j < data_matrix.GetColumns(); j++) {
            row += data_matrix(i, j);
        }
        std::string label = row.substr(0, row.find_last_not_of(' ') + 1);

        label_point_map[label] = i;
        point_label.push_back(label);
    }
}

ParameterInfo C3DFile::getParamInfo(const char *id_str) {
    std::string id_string(id_str);
    assert(id_string.find(":") != std::string::npos);

    std::string group;
    std::string param;

    group = id_string.substr(0, id_string.find(":"));
    param = id_string.substr(id_string.find(":") + 1, id_string.length());

    //	std::cout << "group = " << group << " param = " << param << std::endl;

    Sint8 group_id = -1;

    // search for the group id first as we need to find the parameter with the
    // given id
    std::vector<GroupInfo>::const_iterator group_iter = group_infos.begin();
    while (group_iter != group_infos.end()) {
        if (!strcmp(group.c_str(), group_iter->name)) {
            group_id = group_iter->id;
            break;
        }
        group_iter++;
    }
    assert(group_iter != group_infos.end() && "Could not find group!");

    std::vector<ParameterInfo>::const_iterator param_iter = param_infos.begin();

    while (param_iter != param_infos.end()) {
        if (!strncmp(param.c_str(), param_iter->name, param_iter->name_length) && param_iter->group_id == -group_id) {
            break;
        }
        param_iter++;
    }

    assert(param_iter != param_infos.end() && "Could not find parameter with the specified group");

    /*
	std::cout << "Group id = " << static_cast<int>(group_id)
		<< " param_iter->name = " << param_iter->name
		<< " param_iter->group_id = " << static_cast<int>(param_iter->group_id) << std::endl;
	*/

    return *param_iter;
}

template <typename T>
T C3DFile::getParamGeneric(const char *id_str, T default_value) {
    ParameterInfo param_info;

    param_info = getParamInfo(id_str);

    /*
	std::cout << "param_info.data_type    = " << static_cast<int>(param_info.data_type) << std::endl;
	std::cout << "param_info.n_dimensions = " << static_cast<int>(param_info.n_dimensions) << std::endl;
	std::cout << "param_info.n_dimensions = " << static_cast<int>(param_info.dimensions[0]) << std::endl;
	*/

    if (param_info.n_dimensions != 0 || (param_info.n_dimensions == 1 && param_info.dimensions[0] != 1)) {
        std::cerr << "Invalid query for parameter " << id_str << ": parameter is multidimensional (dim = ";

        int i;
        std::cerr << static_cast<int>(param_info.dimensions[0]);
        for (i = 1; i < param_info.n_dimensions; i++) {
            std::cerr << ", " << static_cast<int>(param_info.dimensions[i]);
        }
        std::cerr << ")." << std::endl;
        return 0;
    }

    assert(param_info.n_dimensions == 0 || (param_info.n_dimensions == 1 && param_info.dimensions[0] == 1));
    assert(check_param_type<T>(param_info.data_type));

    if (param_info.data_type == 1 || param_info.data_type == -1)
        return param_info.char_data[0];
    if (param_info.data_type == 2)
        return param_info.int_data[0];
    if (param_info.data_type == 4)
        return param_info.float_data[0];

    return default_value;
}

}  // namespace crl::mocap
