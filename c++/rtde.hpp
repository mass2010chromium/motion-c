#pragma once
#include <string>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <netdb.h>

#include <boost/unordered_map.hpp>

#include "bytebuffer.h"

int std_connect(int fd, struct sockaddr* info, socklen_t infosz) {
    return connect(fd, info, infosz);
}

using std::string;
using boost::unordered_map;

//from . import serialize

static const float DEFAULT_TIMEOUT = 1.0;

enum Command { RTDE_REQUEST_PROTOCOL_VERSION = 86,      // ascii V
               RTDE_GET_URCONTROL_VERSION = 118,        // ascii v
               RTDE_TEXT_MESSAGE = 77,                  // ascii M
               RTDE_DATA_PACKAGE = 85,                  // ascii U
               RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79, // ascii O
               RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,  // ascii I
               RTDE_CONTROL_PACKAGE_START = 83,         // ascii S
               RTDE_CONTROL_PACKAGE_PAUSE = 80          // ascii P
            };

enum ConnectionState { DISCONNECTED = 0, CONNECTED = 1, STARTED = 2, PAUSED = 3 };
enum DataType { FLOAT, INT, VECTOR3, VECTOR6 };

class RTDE {
    public:
        RTDE(const string& hostname, const int& port=30004) :
            hostname(hostname),
            port(port),
            conn_state(DISCONNECTED),
            sock(-1),
            buf(NULL) {};

        /*
         * Connect to the UR. Sets up the socket and allocates a recv buffer.
         */
        void connect();
        void disconnect();
        bool is_connected();
/**
    def get_controller_version(self):
        cmd = Command.RTDE_GET_URCONTROL_VERSION
        version = self.__sendAndReceive(cmd)
        if version:
            logging.info('Controller version: ' + str(version.major) + '.' + str(version.minor) + '.' + str(version.bugfix)+ '.' + str(version.build))
            if version.major == 3 and version.minor <= 2 and version.bugfix < 19171:
                logging.error("Please upgrade your controller to minimally version 3.2.19171")
                sys.exit()
            return version.major, version.minor, version.bugfix, version.build
        return None, None, None, None

    def negotiate_protocol_version(self):
        cmd = Command.RTDE_REQUEST_PROTOCOL_VERSION
        payload = struct.pack('>H', RTDE_PROTOCOL_VERSION)
        success = self.__sendAndReceive(cmd, payload)
        return success

    def send_input_setup(self, variables, types=[]):
        cmd = Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS
        payload = b','.join([v.encode('ascii') for v in variables])
        result = self.__sendAndReceive(cmd, payload)
        if len(types)!=0 and not self.__list_equals(result.types, types):
            logging.error('Data type inconsistency for input setup: ' +
                     str(types) + ' - ' +
                     str(result.types))
            return None
        result.names = variables
        self.__input_config[result.id] = result
        return serialize.DataObject.create_empty(variables, result.id)

    def send_output_setup(self, variables, types=[], frequency=250):
        cmd = Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS
        payload = struct.pack('>d', frequency)
        payload = payload + b','.join([v.encode('ascii') for v in variables])
        result = self.__sendAndReceive(cmd, payload)
        if len(types)!=0 and not self.__list_equals(result.types, types):
            logging.error('Data type inconsistency for output setup: ' +
                     str(types) + ' - ' +
                     str(result.types))
            return False
        result.names = variables
        self.__output_config = result
        return True

    def send_start(self):
        cmd = Command.RTDE_CONTROL_PACKAGE_START
        success = self.__sendAndReceive(cmd)
        if success:
            logging.info('RTDE synchronization started')
            self.__conn_state = ConnectionState.STARTED
        else:
            logging.error('RTDE synchronization failed to start')
        return success

    def send_pause(self):
        cmd = Command.RTDE_CONTROL_PACKAGE_PAUSE
        success = self.__sendAndReceive(cmd)
        if success:
            logging.info('RTDE synchronization paused')
            self.__conn_state = ConnectionState.PAUSED
        else:
            logging.error('RTDE synchronization failed to pause')
        return success

    def send(self, input_data):
        if self.__conn_state != ConnectionState.STARTED:
            logging.error('Cannot send when RTDE synchronization is inactive')
            return
        if input_data.recipe_id not in self.__input_config:
            logging.error('Input configuration id not found: ' + str(input_data.recipe_id))
            return
        config = self.__input_config[input_data.recipe_id]
        return self.__sendall(Command.RTDE_DATA_PACKAGE, config.pack(input_data))

    def receive(self):
        if self.__output_config is None:
            logging.error('Output configuration not initialized')
            return None
        if self.__conn_state != ConnectionState.STARTED:
            logging.error('Cannot receive when RTDE synchronization is inactive')
            return None
        return self.__recv(Command.RTDE_DATA_PACKAGE)

    def send_message(self, message, source = "Python Client", type = serialize.Message.INFO_MESSAGE):
        cmd = Command.RTDE_TEXT_MESSAGE
        fmt = '>B%dsB%dsB' % (len(message), len(source))
        payload = struct.pack(fmt, len(message), message, len(source), source, type)
        return self.__sendall(cmd, payload)

    def __on_packet(self, cmd, payload):
        if cmd == Command.RTDE_REQUEST_PROTOCOL_VERSION:
            return self.__unpack_protocol_version_package(payload)
        elif cmd == Command.RTDE_GET_URCONTROL_VERSION:
            return self.__unpack_urcontrol_version_package(payload)
        elif cmd == Command.RTDE_TEXT_MESSAGE:
            return self.__unpack_text_message(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
            return self.__unpack_setup_outputs_package(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
            return self.__unpack_setup_inputs_package(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_START:
            return self.__unpack_start_package(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_PAUSE:
            return self.__unpack_pause_package(payload)
        elif cmd == Command.RTDE_DATA_PACKAGE:
            return self.__unpack_data_package(payload, self.__output_config)
        else:
            logging.error('Unknown package command: ' + str(cmd))

    def __sendAndReceive(self, cmd, payload=b''):
        if self.__sendall(cmd, payload):
            return self.__recv(cmd)
        else:
            return None

    def __sendall(self, command, payload=b''):
        fmt = '>HB'
        size = struct.calcsize(fmt) + len(payload)
        buf = struct.pack(fmt, size, command) + payload

        if self.__sock is None:
            logging.error('Unable to send: not connected to Robot')
            return False

        _, writable, _ = select.select([], [self.__sock], [], DEFAULT_TIMEOUT)
        if len(writable):
            self.__sock.sendall(buf)
            return True
        else:
            self.__trigger_disconnected()
            return False

    def has_data(self):
        timeout = 0
        readable, _, _ = select.select([self.__sock], [], [], timeout)
        return len(readable)!=0

    def __recv(self, command):
        while self.is_connected():
            readable, _, xlist = select.select([self.__sock], [], [self.__sock], DEFAULT_TIMEOUT)
            if len(readable):
                more = self.__sock.recv(4096)
                if len(more) == 0:
                    self.__trigger_disconnected()
                    return None
                self.__buf = self.__buf + more

            if len(xlist) or len(readable) == 0: # Effectively a timeout of DEFAULT_TIMEOUT seconds
                logging.info('lost connection with controller')
                self.__trigger_disconnected()
                return None

            # unpack_from requires a buffer of at least 3 bytes
            while len(self.__buf) >= 3:
                # Attempts to extract a packet
                packet_header = serialize.ControlHeader.unpack(self.__buf)

                if len(self.__buf) >= packet_header.size:
                    packet, self.__buf = self.__buf[3:packet_header.size], self.__buf[packet_header.size:]
                    data = self.__on_packet(packet_header.command, packet)
                    if len(self.__buf) >= 3 and command == Command.RTDE_DATA_PACKAGE:
                        next_packet_header = serialize.ControlHeader.unpack(self.__buf)
                        if next_packet_header.command == command:
                            logging.info('skipping package(1)')
                            continue
                    if packet_header.command == command:
                        return data
                    else:
                        logging.info('skipping package(2)')
                else:
                    break
        return None

    def __trigger_disconnected(self):
        logging.info("RTDE disconnected")
        self.disconnect() #clean-up

    def __unpack_protocol_version_package(self, payload):
        if len(payload) != 1:
            logging.error('RTDE_REQUEST_PROTOCOL_VERSION: Wrong payload size')
            return None
        result = serialize.ReturnValue.unpack(payload)
        return result.success

    def __unpack_urcontrol_version_package(self, payload):
        if len(payload) != 16:
            logging.error('RTDE_GET_URCONTROL_VERSION: Wrong payload size')
            return None
        version = serialize.ControlVersion.unpack(payload)
        return version

    def __unpack_text_message(self, payload):
        if len(payload) < 1:
            logging.error('RTDE_TEXT_MESSAGE: No payload')
            return None
        msg = serialize.Message.unpack(payload)
        if(msg.level == serialize.Message.EXCEPTION_MESSAGE or
           msg.level == serialize.Message.ERROR_MESSAGE):
            logging.error(msg.source + ': ' + msg.message)
        elif msg.level == serialize.Message.WARNING_MESSAGE:
            logging.warning(msg.source + ': ' + msg.message)
        elif msg.level == serialize.Message.INFO_MESSAGE:
            logging.info(msg.source + ': ' + msg.message)

    def __unpack_setup_outputs_package(self, payload):
        if len(payload) < 1:
            logging.error('RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS: No payload')
            return None
        output_config = serialize.DataConfig.unpack_recipe(payload)
        return output_config

    def __unpack_setup_inputs_package(self, payload):
        if len(payload) < 1:
            logging.error('RTDE_CONTROL_PACKAGE_SETUP_INPUTS: No payload')
            return None
        input_config = serialize.DataConfig.unpack_recipe(payload)
        return input_config

    def __unpack_start_package(self, payload):
        if len(payload) != 1:
            logging.error('RTDE_CONTROL_PACKAGE_START: Wrong payload size')
            return None
        result = serialize.ReturnValue.unpack(payload)
        return result.success

    def __unpack_pause_package(self, payload):
        if len(payload) != 1:
            logging.error('RTDE_CONTROL_PACKAGE_PAUSE: Wrong payload size')
            return None
        result = serialize.ReturnValue.unpack(payload)
        return result.success

    def __unpack_data_package(self, payload, output_config):
        if output_config is None:
            logging.error('RTDE_DATA_PACKAGE: Missing output configuration')
            return None
        output = output_config.unpack(payload)
        return output

    def __list_equals(self, l1, l2):
        if len(l1) != len(l2):
            return False
        for i in range(len((l1))):
            if l1[i] != l2[i]:
                return False
        return True
*/
#include "rtde_bind.hpp"
        
        ~RTDE();

    private:
        string hostname;
        int port;
        ConnectionState conn_state;
        int sock;
        unordered_map<string, DataType> output_config;
        unordered_map<string, DataType> input_config;
        Buffer* buf;
};

/*
 * Connect to the UR. Sets up the socket and allocates a recv buffer.
 */
void RTDE::connect() {
    if (this->sock >= 0) {
        return;
    }
    struct sockaddr_in host_info;
    host_info.sin_family = AF_INET;
    host_info.sin_port = htons(this->port);

    struct hostent* host_entry = gethostbyname(this->hostname.c_str());
    if (host_entry == NULL) {
        throw std::runtime_error("Unknown Host");
    }
    host_info.sin_addr = *(struct in_addr*) host_entry->h_addr;
    free(host_entry);

    this->sock = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (this->sock < 0) {
        throw std::runtime_error("could not create new socket");
    }
    int yes = 1;
    struct timeval timeout;      
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(this->sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    setsockopt(this->sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(this->sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(this->sock, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    if (std_connect(this->sock, (struct sockaddr*) &host_info, sizeof(host_info)) < 0) {
        throw std::runtime_error("could not connect to UR (did you turn them on? buffoon.)");
    }
    this->conn_state = CONNECTED;
    this->buf = alloc_Buffer(1024);
}

RTDE::~RTDE() {
    free(this->buf);
    if (this->sock >= 0) {
        close(this->sock);
    }
}

/**
class RTDE(object):

    def disconnect(self):
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.__conn_state = ConnectionState.DISCONNECTED

    def is_connected(self):
        return self.__conn_state is not ConnectionState.DISCONNECTED

    def get_controller_version(self):
        cmd = Command.RTDE_GET_URCONTROL_VERSION
        version = self.__sendAndReceive(cmd)
        if version:
            logging.info('Controller version: ' + str(version.major) + '.' + str(version.minor) + '.' + str(version.bugfix)+ '.' + str(version.build))
            if version.major == 3 and version.minor <= 2 and version.bugfix < 19171:
                logging.error("Please upgrade your controller to minimally version 3.2.19171")
                sys.exit()
            return version.major, version.minor, version.bugfix, version.build
        return None, None, None, None

    def negotiate_protocol_version(self):
        cmd = Command.RTDE_REQUEST_PROTOCOL_VERSION
        payload = struct.pack('>H', RTDE_PROTOCOL_VERSION)
        success = self.__sendAndReceive(cmd, payload)
        return success

    def send_input_setup(self, variables, types=[]):
        cmd = Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS
        payload = b','.join([v.encode('ascii') for v in variables])
        result = self.__sendAndReceive(cmd, payload)
        if len(types)!=0 and not self.__list_equals(result.types, types):
            logging.error('Data type inconsistency for input setup: ' +
                     str(types) + ' - ' +
                     str(result.types))
            return None
        result.names = variables
        self.__input_config[result.id] = result
        return serialize.DataObject.create_empty(variables, result.id)

    def send_output_setup(self, variables, types=[], frequency=250):
        cmd = Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS
        payload = struct.pack('>d', frequency)
        payload = payload + b','.join([v.encode('ascii') for v in variables])
        result = self.__sendAndReceive(cmd, payload)
        if len(types)!=0 and not self.__list_equals(result.types, types):
            logging.error('Data type inconsistency for output setup: ' +
                     str(types) + ' - ' +
                     str(result.types))
            return False
        result.names = variables
        self.__output_config = result
        return True

    def send_start(self):
        cmd = Command.RTDE_CONTROL_PACKAGE_START
        success = self.__sendAndReceive(cmd)
        if success:
            logging.info('RTDE synchronization started')
            self.__conn_state = ConnectionState.STARTED
        else:
            logging.error('RTDE synchronization failed to start')
        return success

    def send_pause(self):
        cmd = Command.RTDE_CONTROL_PACKAGE_PAUSE
        success = self.__sendAndReceive(cmd)
        if success:
            logging.info('RTDE synchronization paused')
            self.__conn_state = ConnectionState.PAUSED
        else:
            logging.error('RTDE synchronization failed to pause')
        return success

    def send(self, input_data):
        if self.__conn_state != ConnectionState.STARTED:
            logging.error('Cannot send when RTDE synchronization is inactive')
            return
        if input_data.recipe_id not in self.__input_config:
            logging.error('Input configuration id not found: ' + str(input_data.recipe_id))
            return
        config = self.__input_config[input_data.recipe_id]
        return self.__sendall(Command.RTDE_DATA_PACKAGE, config.pack(input_data))

    def receive(self):
        if self.__output_config is None:
            logging.error('Output configuration not initialized')
            return None
        if self.__conn_state != ConnectionState.STARTED:
            logging.error('Cannot receive when RTDE synchronization is inactive')
            return None
        return self.__recv(Command.RTDE_DATA_PACKAGE)

    def send_message(self, message, source = "Python Client", type = serialize.Message.INFO_MESSAGE):
        cmd = Command.RTDE_TEXT_MESSAGE
        fmt = '>B%dsB%dsB' % (len(message), len(source))
        payload = struct.pack(fmt, len(message), message, len(source), source, type)
        return self.__sendall(cmd, payload)

    def __on_packet(self, cmd, payload):
        if cmd == Command.RTDE_REQUEST_PROTOCOL_VERSION:
            return self.__unpack_protocol_version_package(payload)
        elif cmd == Command.RTDE_GET_URCONTROL_VERSION:
            return self.__unpack_urcontrol_version_package(payload)
        elif cmd == Command.RTDE_TEXT_MESSAGE:
            return self.__unpack_text_message(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
            return self.__unpack_setup_outputs_package(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
            return self.__unpack_setup_inputs_package(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_START:
            return self.__unpack_start_package(payload)
        elif cmd == Command.RTDE_CONTROL_PACKAGE_PAUSE:
            return self.__unpack_pause_package(payload)
        elif cmd == Command.RTDE_DATA_PACKAGE:
            return self.__unpack_data_package(payload, self.__output_config)
        else:
            logging.error('Unknown package command: ' + str(cmd))

    def __sendAndReceive(self, cmd, payload=b''):
        if self.__sendall(cmd, payload):
            return self.__recv(cmd)
        else:
            return None

    def __sendall(self, command, payload=b''):
        fmt = '>HB'
        size = struct.calcsize(fmt) + len(payload)
        buf = struct.pack(fmt, size, command) + payload

        if self.__sock is None:
            logging.error('Unable to send: not connected to Robot')
            return False

        _, writable, _ = select.select([], [self.__sock], [], DEFAULT_TIMEOUT)
        if len(writable):
            self.__sock.sendall(buf)
            return True
        else:
            self.__trigger_disconnected()
            return False

    def has_data(self):
        timeout = 0
        readable, _, _ = select.select([self.__sock], [], [], timeout)
        return len(readable)!=0

    def __recv(self, command):
        while self.is_connected():
            readable, _, xlist = select.select([self.__sock], [], [self.__sock], DEFAULT_TIMEOUT)
            if len(readable):
                more = self.__sock.recv(4096)
                if len(more) == 0:
                    self.__trigger_disconnected()
                    return None
                self.__buf = self.__buf + more

            if len(xlist) or len(readable) == 0: # Effectively a timeout of DEFAULT_TIMEOUT seconds
                logging.info('lost connection with controller')
                self.__trigger_disconnected()
                return None

            # unpack_from requires a buffer of at least 3 bytes
            while len(self.__buf) >= 3:
                # Attempts to extract a packet
                packet_header = serialize.ControlHeader.unpack(self.__buf)

                if len(self.__buf) >= packet_header.size:
                    packet, self.__buf = self.__buf[3:packet_header.size], self.__buf[packet_header.size:]
                    data = self.__on_packet(packet_header.command, packet)
                    if len(self.__buf) >= 3 and command == Command.RTDE_DATA_PACKAGE:
                        next_packet_header = serialize.ControlHeader.unpack(self.__buf)
                        if next_packet_header.command == command:
                            logging.info('skipping package(1)')
                            continue
                    if packet_header.command == command:
                        return data
                    else:
                        logging.info('skipping package(2)')
                else:
                    break
        return None

    def __trigger_disconnected(self):
        logging.info("RTDE disconnected")
        self.disconnect() #clean-up

    def __unpack_protocol_version_package(self, payload):
        if len(payload) != 1:
            logging.error('RTDE_REQUEST_PROTOCOL_VERSION: Wrong payload size')
            return None
        result = serialize.ReturnValue.unpack(payload)
        return result.success

    def __unpack_urcontrol_version_package(self, payload):
        if len(payload) != 16:
            logging.error('RTDE_GET_URCONTROL_VERSION: Wrong payload size')
            return None
        version = serialize.ControlVersion.unpack(payload)
        return version

    def __unpack_text_message(self, payload):
        if len(payload) < 1:
            logging.error('RTDE_TEXT_MESSAGE: No payload')
            return None
        msg = serialize.Message.unpack(payload)
        if(msg.level == serialize.Message.EXCEPTION_MESSAGE or
           msg.level == serialize.Message.ERROR_MESSAGE):
            logging.error(msg.source + ': ' + msg.message)
        elif msg.level == serialize.Message.WARNING_MESSAGE:
            logging.warning(msg.source + ': ' + msg.message)
        elif msg.level == serialize.Message.INFO_MESSAGE:
            logging.info(msg.source + ': ' + msg.message)

    def __unpack_setup_outputs_package(self, payload):
        if len(payload) < 1:
            logging.error('RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS: No payload')
            return None
        output_config = serialize.DataConfig.unpack_recipe(payload)
        return output_config

    def __unpack_setup_inputs_package(self, payload):
        if len(payload) < 1:
            logging.error('RTDE_CONTROL_PACKAGE_SETUP_INPUTS: No payload')
            return None
        input_config = serialize.DataConfig.unpack_recipe(payload)
        return input_config

    def __unpack_start_package(self, payload):
        if len(payload) != 1:
            logging.error('RTDE_CONTROL_PACKAGE_START: Wrong payload size')
            return None
        result = serialize.ReturnValue.unpack(payload)
        return result.success

    def __unpack_pause_package(self, payload):
        if len(payload) != 1:
            logging.error('RTDE_CONTROL_PACKAGE_PAUSE: Wrong payload size')
            return None
        result = serialize.ReturnValue.unpack(payload)
        return result.success

    def __unpack_data_package(self, payload, output_config):
        if output_config is None:
            logging.error('RTDE_DATA_PACKAGE: Missing output configuration')
            return None
        output = output_config.unpack(payload)
        return output

    def __list_equals(self, l1, l2):
        if len(l1) != len(l2):
            return False
        for i in range(len((l1))):
            if l1[i] != l2[i]:
                return False
        return True
*/
#include "rtde_bind.hpp"
