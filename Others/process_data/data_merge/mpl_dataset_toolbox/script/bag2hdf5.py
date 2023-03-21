import argparse
import h5py
import hdf5plugin
import progressbar
from prophesee_event_msgs.msg import EventArray
import rosbag
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert ROS bag to hdf5 for event stream')
    parser.add_argument('topic', type=str, help='ROS topic for event stream')
    parser.add_argument('input_path', type=str, help='Input ROS bag path')
    parser.add_argument('output_path', type=str, help='Output HDF5 file path')
    args = parser.parse_args()
    
    event_topic = args.topic
    input_data_path = args.input_path
    bag = rosbag.Bag(input_data_path, 'r')
    output_data_path = args.output_path
    event_file = h5py.File(output_data_path, 'w')
    
    event_curr_idx = 0
    event_ms_idx = 0
    chunk_size = 100000
    receive_event = False
    event_x = []
    event_y = []
    event_p = []
    event_t = []
    event_ms_to_idx = []
    bar = progressbar.ProgressBar(maxval=bag.size).start()
    for topic, msg, t in bag.read_messages(event_topic):
        bar.update(bag._file.tell())
        if (topic == event_topic):
            # Add this event into a temporary numpy holder
            if not receive_event:
                receive_event = True
                event_t_offset = msg.events[0].ts
            for event in msg.events:
                event_x.append(event.x)
                event_y.append(event.y)
                event_p.append(event.polarity)
                event_t.append(int((event.ts.to_nsec() - event_t_offset.to_nsec()) / 1e3))
                while (event_t[-1] >= 1000 * event_ms_idx):
                    event_ms_to_idx.append(event_curr_idx)
                    event_ms_idx += 1
                event_curr_idx += 1
            # Flush the holder temporarily
            if (len(event_x) >= chunk_size):
                if ('events/t' not in event_file):
                    event_file.create_dataset('events/x', dtype='u2', maxshape=(None, ), data=np.array(event_x, dtype='u2'), **hdf5plugin.Zstd())
                    event_file.create_dataset('events/y', dtype='u2', maxshape=(None, ), data=np.array(event_y, dtype='u2'), **hdf5plugin.Zstd())
                    event_file.create_dataset('events/p', dtype='u1', maxshape=(None, ), data=np.array(event_p, dtype='u1'), **hdf5plugin.Zstd())
                    event_file.create_dataset('events/t', dtype='u4', maxshape=(None, ), data=np.array(event_t, dtype='u4'), **hdf5plugin.Zstd())
                else:
                    old_size = event_file['events/t'].shape[0]
                    new_size = len(event_t) + old_size
                    event_file['events/x'].resize((new_size, ))
                    event_file['events/x'][old_size:] = data=np.array(event_x, dtype='u2')
                    event_file['events/y'].resize((new_size, ))
                    event_file['events/y'][old_size:] = data=np.array(event_y, dtype='u2')
                    event_file['events/p'].resize((new_size, ))
                    event_file['events/p'][old_size:] = data=np.array(event_p, dtype='u1')
                    event_file['events/t'].resize((new_size, ))
                    event_file['events/t'][old_size:] = data=np.array(event_t, dtype='u4')
                event_x.clear()
                event_y.clear()
                event_p.clear()
                event_t.clear()
    # Flush the remaining data on holder
    old_size = event_file['events/t'].shape[0]
    new_size = len(event_t) + old_size
    event_file['events/x'].resize((new_size, ))
    event_file['events/x'][old_size:] = data=np.array(event_x, dtype='u2')
    event_file['events/y'].resize((new_size, ))
    event_file['events/y'][old_size:] = data=np.array(event_y, dtype='u2')
    event_file['events/p'].resize((new_size, ))
    event_file['events/p'][old_size:] = data=np.array(event_p, dtype='u1')
    event_file['events/t'].resize((new_size, ))
    event_file['events/t'][old_size:] = data=np.array(event_t, dtype='u4')
    event_file.create_dataset('ms_to_idx', dtype='u8', data=np.array(event_ms_to_idx, dtype='u8'), **hdf5plugin.Zstd())
    event_file.create_dataset('t_offset', shape = (1,), dtype='i8', data=int(event_t_offset.to_nsec() / 1e3), **hdf5plugin.Zstd())
    bar.finish()
    event_file.close()
