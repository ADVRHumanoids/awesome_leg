#include "calib_utils.hpp"

using namespace CalibUtils;
using namespace SignProcUtils;

void AuxSigDecoder::on_js_signal_received(const xbot_msgs::JointState& js_sig)
{
  _jnt_names = js_sig.name;

  _is_first_js_sig = false; // to signal that we now have read the joint names
}

void AuxSigDecoder::on_aux_signal_received(const xbot_msgs::CustomState& aux_sig)
{
    _was_aux_msg_received = true;

    if (!_is_first_js_sig)
    {

        auto remapped_aux_tuple = aux_mapper(aux_sig); // remapping aux types

        std::vector<double> ordered_vals = std::get<1>(remapped_aux_tuple); // ordered as _jnt_names

        double* ptr = &ordered_vals[0];

    //    Eigen::Map<Eigen::VectorXd> ordered_iq(ptr, ordered_vals.size());

    //    _iq_out_fb = ordered_iq;

        _iq_out_fb = Eigen::Map<Eigen::VectorXd>(ptr, ordered_vals.size());

    }


}

template <typename T, typename t_v >
int AuxSigDecoder::find_index(std::vector<T> input_v, t_v value)
{
    /**
    Finds the index of a value in an array.

    @param input_v Input vector.
    @param value Value to be searched within the input vector.

    @return  The index of the element (-1 if not present).
    */

    auto it = find(input_v.begin(), input_v.end(), value);

    // If element was found
    if (it != input_v.end())
    {
        // calculating the index
        int index = it - input_v.begin();
        return index;
    }
    else
    {
        // The element is not present in the vector
        return -1;
    }
}

template <typename T>
std::vector<int> AuxSigDecoder::map_indices(std::vector<T> input_v1, std::vector<T> input_v2)
{
    /**
    Maps the elements of the first input array to the second.

    @param input_v1 First input vector.
    @param input_v2 Second input vector.

    @return  A vector of indices. indices_map[i] contains the index where the i-th element of input_v1 is present in input_v2.

    */

    int v1_size = input_v1.size(); //initialize output
    std::vector<int> indices_map(v1_size, -1);

    // Here insert a check (size of input_v1 must equal size of input_v2)
    for (int i = 0; i < input_v1.size(); i++)
    {
        indices_map[i] = find_index(input_v2, input_v1[i] );
    }
    return indices_map;
}

int AuxSigDecoder::get_aux_type_code(std::string msg_type)
{
    /**
    Computes the code ID associated with a given message type and saves this code to the .mat file,
    so that other programs can interpret the signal-type information. To avoid the use of strings, a unique _aux_code suffix is employed.

    @param msg_type The message type name (std::string).
    @return The message code to be associated with the message type name
    */

    int msg_code;

    if (_aux_msg_type_map.find(msg_type) == _aux_msg_type_map.end()) // msg_type not present
    {
        _aux_types_encode_number++; // increment last signal code number by 1
        _aux_msg_type_map.insert({msg_type, _aux_types_encode_number}); // put the code in the dictionary
    }

    msg_code = _aux_msg_type_map.at(msg_type); // read the key associated with the msg_type from the aux code map

    return msg_code;
}

std::tuple<std::vector<int>, std::vector<double>> AuxSigDecoder::aux_mapper(const xbot_msgs::CustomState& aux_sig)
{

    /**
    Loops through the chains and their joints and, based on the received message, assigns the IDs associated with each message type.

    @param msg Input message
    @return A vector of signal IDs with dimension (number of chains)*(number of joints)
    */

    int n_jnts = aux_sig.name.size(); // number of joints

    std::vector<double> msg_value_remapped(n_jnts, -1.0); // output vector for msg values
    std::vector<int> msg_type_remapped(n_jnts, -1.0); // output vector for msg types

    if (_is_first_aux_sig)
    { // this runs only the first time an aux message is received
      // (joint mapping is assumed to be constant throughout the life of this
      // object)

        _indices = map_indices(_jnt_names, aux_sig.name);

        _is_first_aux_sig = false; // mapping not needed anymore
    }

    for (int i = 0; i < n_jnts; i++) // mapping
    {
        if (aux_sig.type[i].find(_iq_sig_basename) != std::string::npos)
        { // we are reading the iq aux type

            int encoded_type = get_aux_type_code(aux_sig.type[i]);
            msg_type_remapped[_indices[i]] = encoded_type;
            msg_value_remapped[_indices[i]] = aux_sig.value[i];

        }

    }

    return make_tuple(msg_type_remapped, msg_value_remapped);
}
