#include "mocap/c3d/c3dutils.h"

#include <iostream>

using namespace std;

namespace crl::mocap {

void print_c3dheaderinfo(const C3DHeader &header) {
    if (header.c3d_id != 80) {
        cout << "Invalid C3D id at byte 2! Is: 0x" << hex << static_cast<int>(header.c3d_id) << " but should be: 0x50" << endl;
        assert(header.c3d_id == 80);
    }

    cout << "first_parameter         : 0x" << hex << static_cast<int>(header.first_parameter) << endl;
    cout << "c3d_id                  : 0x" << hex << static_cast<int>(header.c3d_id) << endl;
    cout << dec;
    cout << "num_markers             : " << header.num_markers << endl;
    cout << "analog_channels         : " << header.analog_channels << endl;
    cout << "first_frame             : " << header.first_frame << endl;
    cout << "last_frame              : " << header.last_frame << endl;
    cout << "max_interpolation_gap   : " << header.max_interpolation_gap << endl;
    cout << "scale_factor            : " << scientific << header.scale_factor << endl;
    cout << "start_record            : " << header.start_record << endl;
    cout << "analog_channels_samples : " << header.analog_channels_samples << endl;
    cout << "video_sampling_rate     : " << header.video_sampling_rate << endl;
    //	cout << "reserved_1[134]        : " << reserved_1[134]<< endl;
    cout << "key_value_label         : " << header.key_value_label << endl;
    cout << "block_label_start       : " << header.block_label_start << endl;
    cout << "key_value_char_4_events : " << header.key_value_char_4_events << endl;
    cout << "num_events              : " << header.num_events << endl;
    //	cout << "reserved_2             : " << reserved_2<< endl;
    //	cout << "event_times[18]        :" << event_times[18]<< endl;
    //	cout << "event_display_flags[18]:" << event_display_flags[18]<< endl;
    //	cout << "reserved_3             :" << reserved_3<< endl;
    //	cout << "event_labels[18][4]    :" << event_labels[18][4]<< endl;
    //	cout << "reserved_4[22]         :" << reserved_4[22]<< endl;
}

void print_group_info(const GroupInfo &group_info) {
    cout << "Group: " << group_info.name << "(" << dec << static_cast<int>(group_info.name_length) << ") "
         << "ID " << static_cast<int>(group_info.id) << ": " << group_info.description << "(" << static_cast<int>(group_info.descr_length) << ") "
         << "next: " << group_info.next_offset << endl;
}

void print_parameter_info(const ParameterInfo &parameter_info) {
    cout << "Parameter: " << parameter_info.name << "(" << static_cast<int>(parameter_info.name_length) << ")" << endl
         << "  "
         << "locked: " << parameter_info.locked << endl
         << "  "
         << "gid   : " << static_cast<int>(parameter_info.group_id) << endl
         << "  "
         << "type  : " << static_cast<int>(parameter_info.data_type) << endl
         << "  "
         << "next  : " << static_cast<int>(parameter_info.next_offset) << endl
         << "  "
         << "#dim  : " << static_cast<int>(parameter_info.n_dimensions) << endl;

    int i;
    for (i = 0; i < parameter_info.n_dimensions; i++) {
        cout << "  "
             << "dim[" << i << "]: " << static_cast<int>(parameter_info.dimensions[i]) << endl;
    }

    if (parameter_info.data_type == 1 || parameter_info.data_type == -1)
        cout << "  "
             << "val   : " << parameter_info.char_data[0] << endl;
    if (parameter_info.data_type == 2)
        cout << "  "
             << "val   : " << static_cast<Sint16>(parameter_info.int_data[0]) << endl;
    if (parameter_info.data_type == 4)
        cout << "  "
             << "val   : " << parameter_info.float_data[0] << endl;

    cout << "  "
         << "desc  : " << parameter_info.description << " (" << static_cast<int>(parameter_info.descr_length) << ")" << endl;
}

void print_event_info(const EventInfo &event_info) {
    cout << "Event: " << event_info.label << endl;
    cout << "  "
         << "context: " << event_info.context << endl;
    cout << "  "
         << "label  : " << event_info.label << endl;
    cout << "  "
         << "minutes: " << event_info.time_minutes << endl;
    cout << "  "
         << "seconds: " << event_info.time_seconds << endl;
}

}  // namespace crl::mocap