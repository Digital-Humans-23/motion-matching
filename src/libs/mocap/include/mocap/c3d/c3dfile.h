#ifndef C3D_FILE
#define C3D_FILE

#include <fstream>
#include <map>
#include <vector>

#include "c3dtypes.h"

namespace crl::mocap {

struct C3DFile {
    bool load(const char* filename);
    FloatMarkerData getMarkerTrajectories(const char* point_name_str);
    size_t getEventCount();
    EventInfo getEventInfo(size_t index);

    std::string getParamString(const char* id_str);
    Sint8 getParamSint8(const char* id_str);
    Sint16 getParamSint16(const char* id_str);
    float getParamFloat(const char* id_str);

    C3DHeader header;
    std::vector<ParameterInfo> param_infos;
    std::vector<GroupInfo> group_infos;
    std::vector<std::vector<FramePointInfo> > point_data;

    std::vector<std::string> point_label;
    std::map<std::string, Sint16> label_point_map;
    std::vector<FloatMarkerData> float_point_data;
    std::map<int, int> group_id_to_index_map;
    bool uses_integer_data;

    void readParameterSection(std::ifstream& data_stream);
    GroupInfo readGroupInfo(std::ifstream& data_stream);
    ParameterInfo readParameterInfo(std::ifstream& datastream);
    void readPointSection(std::ifstream& datastream);
    void fillPointLabelMap();
    ParameterInfo getParamInfo(const char* id_str);

    template <typename T>
    T getParamGeneric(const char* id_str, T default_value);
};

}  // namespace crl::mocap

#endif /* C3D_FILE */
