#ifndef C3DTYPES
#define C3DTYPES

#include <assert.h>
#include <ctype.h>
#include <stdint.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

namespace crl::mocap {

typedef int8_t Sint8;
typedef uint8_t Uint8;
typedef int16_t Sint16;
typedef uint16_t Uint16;
typedef int32_t Sint32;
typedef uint32_t Uint32;

/** \brief see p. 32
 */
struct C3DHeader {
    Uint8 first_parameter;           // 1
    Uint8 c3d_id;                    // 1
    Uint16 num_markers;              // 2
    Uint16 analog_channels;          // 3
    Uint16 first_frame;              // 4
    Uint16 last_frame;               // 5
    Uint16 max_interpolation_gap;    // 6
    float scale_factor;              // 7 - 8
    Uint16 start_record;             // 9
    Uint16 analog_channels_samples;  // 10
    float video_sampling_rate;       // 11 - 12
    Uint16 reserved_1[135];          // 13 - 147
    Uint16 key_value_label;          // 148
    Uint16 block_label_start;        // 149
    Uint16 key_value_char_4_events;  // 150
    Uint16 num_events;               // 151
    Uint16 reserved_2;               // 152
    float event_times[18];           // 153 - 188
    Uint8 event_display_flags[18];   // 189 - 197
    Uint16 reserved_3;               // 198
    Uint8 event_labels[18][4];       // 199 - 234
    Uint16 reserved_4[22];           // 235 - 256
};

struct ParameterHeader {
    Uint8 reserved_1;
    Uint8 reserved_2;
    Uint8 num_parameter_blocks;
    Uint8 processor_type;
};

struct GroupInfo {
    Sint8 name_length;
    Sint8 id;
    char *name;
    Sint16 next_offset;
    Sint8 descr_length;
    char *description;
    bool locked;

    GroupInfo() : name_length(-1), id(-1), name(NULL), next_offset(-1), descr_length(-1), description(NULL), locked(false) {}
    ~GroupInfo() {
        if (name)
            delete[] name;
        if (description)
            delete[] description;
    }
    GroupInfo(const GroupInfo &info)
        : name_length(info.name_length),
          id(info.id),
          name(NULL),
          next_offset(info.next_offset),
          descr_length(info.descr_length),
          description(NULL),
          locked(info.locked) {
        // we also need to copy the '\0' character at the end!
        name = new char[name_length + 1];
        memcpy(name, info.name, sizeof(char) * (name_length + 1));

        // we also need to copy the '\0' character at the end!
        description = new char[descr_length + 1];
        memcpy(description, info.description, sizeof(char) * (descr_length + 1));
    }
    GroupInfo &operator=(const GroupInfo &info) {
        if (this != &info) {
            if (name)
                delete[] name;
            if (description)
                delete[] description;

            name_length = info.name_length;
            id = info.id;
            name = NULL;
            next_offset = info.next_offset;
            descr_length = info.descr_length;
            description = NULL;
            locked = info.locked;

            // we also need to copy the '\0' character at the end!
            name = new char[name_length + 1];
            memcpy(name, info.name, sizeof(char) * (name_length + 1));

            // we also need to copy the '\0' character at the end!
            description = new char[descr_length + 1];
            memcpy(description, info.description, sizeof(char) * (descr_length + 1));
        }
        return *this;
    }
};

struct ParameterInfo {
    Sint8 name_length;
    Sint8 group_id;
    char *name;
    Sint16 next_offset;
    Sint8 data_type;
    Uint8 n_dimensions;
    Uint8 *dimensions;

    char *char_data;
    Sint16 *int_data;
    float *float_data;

    Sint8 descr_length;
    char *description;

    bool locked;

    ParameterInfo()
        : name_length(-1),
          group_id(-1),
          name(NULL),
          next_offset(-1),
          data_type(-1),
          n_dimensions(-1),
          dimensions(NULL),

          char_data(NULL),
          int_data(NULL),
          float_data(NULL),

          descr_length(-1),
          description(NULL),
          locked(false) {}
    ~ParameterInfo() {
        if (name)
            delete[] name;
        if (dimensions)
            delete[] dimensions;

        if (char_data)
            delete[] char_data;
        if (int_data)
            delete[] int_data;
        if (float_data)
            delete[] float_data;

        if (description)
            delete[] description;
    }

    ParameterInfo(const ParameterInfo &info)
        : name_length(info.name_length),
          group_id(info.group_id),
          name(NULL),
          next_offset(info.next_offset),
          data_type(info.data_type),
          n_dimensions(info.n_dimensions),
          dimensions(NULL),

          char_data(NULL),
          int_data(NULL),
          float_data(NULL),

          descr_length(info.descr_length),
          description(NULL),
          locked(info.locked) {
        // we also need to copy the '\0' character at the end!
        name = new char[name_length + 1];
        memcpy(name, info.name, sizeof(char) * (name_length + 1));

        if (info.dimensions == 0) {
            // if dim == 0 the data comes right away
            if (info.data_type == 1 || info.data_type == -1) {
                char_data = new char[1];
                memcpy(char_data, info.char_data, sizeof(char));
            } else if (info.data_type == 2) {
                int_data = new Sint16[1];
                memcpy(int_data, info.int_data, sizeof(Sint16));
            } else if (info.data_type == 4) {
                float_data = new float[1];
                memcpy(float_data, info.float_data, sizeof(float));
            }
        } else {
            // we first have to read out the dimensions of the values and then we can
            // store the values
            dimensions = new Uint8[info.n_dimensions];

            memcpy(dimensions, info.dimensions, sizeof(Uint8) * n_dimensions);

            int count = 1;
            int i;
            for (i = 0; i < n_dimensions; i++) {
                count *= dimensions[i];
            }

            if (data_type == 1 || data_type == -1) {
                char_data = new char[count];
                memcpy(char_data, info.char_data, sizeof(char) * count);
            } else if (info.data_type == 2) {
                int_data = new Sint16[count];
                memcpy(int_data, info.int_data, sizeof(Sint16) * count);
            } else if (info.data_type == 4) {
                float_data = new float[count];
                memcpy(float_data, info.float_data, sizeof(float) * count);
            }
        }
    }

    // Assignment operator
    ParameterInfo &operator=(const ParameterInfo &info) {
        if (this != &info) {
            delete[] name;
            delete[] dimensions;

            if (char_data)
                delete[] char_data;
            if (int_data)
                delete[] int_data;
            if (float_data)
                delete[] float_data;

            delete[] description;

            name_length = info.name_length;
            group_id = info.group_id;
            name = NULL;
            next_offset = info.next_offset;
            data_type = info.data_type;
            n_dimensions = info.n_dimensions;
            dimensions = NULL;

            char_data = NULL;
            int_data = NULL;
            float_data = NULL;

            descr_length = info.descr_length;
            description = NULL;
            locked = info.locked;

            // Copy the data
            // we also need to copy the '\0' character at the end!
            name = new char[name_length + 1];
            memcpy(name, info.name, sizeof(char) * (name_length + 1));

            if (info.dimensions == 0) {
                // if dim == 0 the data comes right away
                if (info.data_type == 1 || info.data_type == -1) {
                    char_data = new char[1];
                    memcpy(char_data, info.char_data, sizeof(char));
                } else if (info.data_type == 2) {
                    int_data = new Sint16[1];
                    memcpy(int_data, info.int_data, sizeof(Sint16));
                } else if (info.data_type == 4) {
                    float_data = new float[1];
                    memcpy(float_data, info.float_data, sizeof(float));
                }
            } else {
                // we first have to read out the dimensions of the values and then we can
                // store the values
                dimensions = new Uint8[info.n_dimensions];

                memcpy(dimensions, info.dimensions, sizeof(Uint8) * n_dimensions);

                int count = 1;
                int i;
                for (i = 0; i < n_dimensions; i++) {
                    count *= dimensions[i];
                }

                if (info.data_type == 1 || info.data_type == -1) {
                    char_data = new char[count];
                    memcpy(char_data, info.char_data, sizeof(char) * count);
                } else if (info.data_type == 2) {
                    int_data = new Sint16[count];
                    memcpy(int_data, info.int_data, sizeof(Sint16) * count);
                } else if (info.data_type == 4) {
                    float_data = new float[count];
                    memcpy(float_data, info.float_data, sizeof(float) * count);
                }
            }
        }

        return *this;
    }
};

struct FramePointIntInfo {
    Sint16 x;
    Sint16 y;
    Sint16 z;
    Uint8 cameras;
    Uint8 residual;
};

struct FramePointInfo {
    float x;
    float y;
    float z;
    Uint8 cameras;
    Uint8 residual;
};

struct FloatMarkerData {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;

    std::vector<Uint8> cameras;
    std::vector<Uint8> residual;
};

template <typename T>
class DataMatrix {
public:
    DataMatrix() : raw_data(NULL), rows(0), columns(0){};
    DataMatrix(unsigned int r, unsigned int c) : rows(r), columns(c) {
        raw_data = new T[rows * columns];
    }
    DataMatrix(const DataMatrix &data) : rows(data.rows), columns(data.columns) {
        raw_data = new T[rows * columns];
        memcpy(raw_data, data.raw_data, sizeof(T) * rows * columns);
    }
    DataMatrix &operator=(const DataMatrix &data) {
        if (this != &data) {
            if (raw_data)
                delete[] raw_data;
            rows = data.rows;
            columns = data.columns;

            raw_data = new char[rows * columns];
            memcpy(raw_data, data.raw_data, sizeof(T) * rows * columns);
        }
        return *this;
    }
    ~DataMatrix() {
        if (raw_data)
            delete[] raw_data;
    }
    T &operator()(const unsigned i, const unsigned j) {
        return raw_data[i * columns + j];
    }
    T operator()(const unsigned i, const unsigned j) const {
        return raw_data[i * columns + j];
    }
    T *GetRawPtr() {
        return raw_data;
    }
    void CopyFrom(T *src) {
        memcpy(raw_data, src, sizeof(T) * rows * columns);
    }
    size_t GetRows() const {
        return rows;
    }
    size_t GetColumns() const {
        return columns;
    }

private:
    T *raw_data;
    size_t rows;
    size_t columns;
};

template <typename T>
inline std::ostream &operator<<(std::ostream &output, const DataMatrix<T> &data) {
    unsigned int i, j;
    for (i = 0; i < data.GetRows(); i++) {
        for (j = 0; j < data.GetColumns(); j++) {
            output << data(i, j);
        }
        output << std::endl;
    }

    return output;
}

struct EventInfo {
    EventInfo() : label(""), context(""), time_minutes(0.f), time_seconds(0.f) {}

    std::string label;
    std::string context;
    float time_minutes;
    float time_seconds;
};

}  // namespace crl::mocap

#endif /* C3DTYPES */
