"""
A script which creates a node that subscribes to the topic '/tf' and graphs the recorded
translations and rotations after script interupted (ctrl+c)


    2024 - Jakob Vilinsky - IASRL
"""


import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import sys
import os.path
import dearpygui.dearpygui as dpg
import dearpygui.demo as demo

class ViconSubscriber(Node):

    def __init__(self):
        super().__init__('vicon_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10
        )
        self.subscription

        # self.path = '~/Downloads'
        # self.txtfilename = os.path.join(self.path,sys.argv[1]+".txt")
        # self.file = open(self.txtfilename, "w")

        self.counter = 0

        self.Sec = []
        self.Nanosec = []
        self.Translation_x = []
        self.Translation_y = []
        self.Translation_z = []
        self.Rotation_x = []
        self.Rotation_y = []
        self.Rotation_z = []


    def listener_callback(self, data):
        self.get_logger().info('---------------------')
        self.get_logger().info('Sec: "%i"' % data.transforms[0].header.stamp.sec)
        self.get_logger().info('Nanosec: "%i"' % data.transforms[0].header.stamp.nanosec)
        self.get_logger().info('Translation x: "%f"' % data.transforms[0].transform.translation.x)
        self.get_logger().info('Translation y: "%f"' % data.transforms[0].transform.translation.y)
        self.get_logger().info('Translation z: "%f"' % data.transforms[0].transform.translation.z)
        self.get_logger().info('Rotation x: "%f"' % data.transforms[0].transform.rotation.x)
        self.get_logger().info('Rotation y: "%f"' % data.transforms[0].transform.rotation.y)
        self.get_logger().info('Rotation z: "%f"' % data.transforms[0].transform.rotation.z)
        self.get_logger().info('---------------------')

        self.Sec.append(data.transforms[0].header.stamp.sec)
        self.Nanosec.append(data.transforms[0].header.stamp.nanosec)
        self.Translation_x.append(data.transforms[0].transform.translation.x)
        self.Translation_y.append(data.transforms[0].transform.translation.y)
        self.Translation_z.append(data.transforms[0].transform.translation.z)
        self.Rotation_x.append(data.transforms[0].transform.rotation.x)
        self.Rotation_y.append(data.transforms[0].transform.rotation.y)
        self.Rotation_z.append(data.transforms[0].transform.rotation.z)

        self.counter += 1

    def getData(self):
        return self.Sec, self.Nanosec, self.Translation_x, self.Translation_y, self.Translation_x, \
        self.Rotation_x, self.Rotation_y, self.Rotation_z 


        

def main(args=None):
    try:
        rclpy.init(args=args)

        vicon_subscriber = ViconSubscriber()

        rclpy.spin(vicon_subscriber)

        vicon_subscriber.destroy_node()
        rclpy.shutdown()

    except:
        #When script ends graph data (press ctrl+c)
        Sec, Nanosec, Translation_x, Translation_y, Translation_z, Rotation_x, Rotation_y, Rotation_z = vicon_subscriber.getData()

        #Concatate Seconds and Nanoseconds and undjust time to start at 0
        Time_unajusted = []
        Time = []
        for i in range(len(Sec)):
            Time_unajusted.append(float(f'{Sec[i]}.{Nanosec[i]}'))
            Time_adjust = float(f'{Sec[0]}.{Nanosec[0]}')
            Time.append(Time_unajusted[i] - Time_adjust)
        
        #Plot data using DearPyGui

        def _config(sender, keyword, user_data):
            widget_type = dpg.get_item_type(sender)
            items = user_data

            if widget_type == "mvAppItemType::mvRadioButton":
                value = True
            else:
                keyword = dpg.get_item_label(sender)
                value = dpg.get_value(sender)

            if isinstance(user_data, list):
                for item in items:
                    dpg.configure_item(item, **{keyword: value})
            else:
                dpg.configure_item(items, **{keyword: value})
        
        def _add_config_options(item, columns, *names, **kwargs):
            if columns == 1:
                if 'before' in kwargs:
                    for name in names:
                        dpg.add_checkbox(label=name, callback=_config, user_data=item, before=kwargs['before'], default_value=dpg.get_item_configuration(item)[name])
                else:
                    for name in names:
                        dpg.add_checkbox(label=name, callback=_config, user_data=item, default_value=dpg.get_item_configuration(item)[name])
            else:
                if 'before' in kwargs:
                    dpg.push_container_stack(dpg.add_table(header_row=False, before=kwargs['before']))
                else:
                    dpg.push_container_stack(dpg.add_table(header_row=False))

                for i in range(columns):
                    dpg.add_table_column()

                for i in range((len(names)+(columns - 1))//columns):
                    with dpg.table_row():
                        for j in range(columns):
                            if (i*columns + j) >= len(names): 
                                break
                            dpg.add_checkbox(label=names[i*columns + j], 
                                                callback=_config, user_data=item, 
                                                default_value=dpg.get_item_configuration(item)[names[i*columns + j]])
                dpg.pop_container_stack()

        dpg.create_context()

        with dpg.window(tag="Primary Window"):
            # create plot
            with dpg.subplots(3,3,label="",width=-1,height=-1) as subplot_id:

                dpg.add_plot_legend()
                with dpg.plot(label="Translation x"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="")
                    with dpg.plot_axis(dpg.mvYAxis, label="Distance (meters)"):
                        dpg.add_line_series(Time,Translation_x,label="")

                with dpg.plot(label="Translation y"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="")
                    with dpg.plot_axis(dpg.mvYAxis, label=""):
                        dpg.add_line_series(Time,Translation_y,label="")

                with dpg.plot(label="Translation z"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="")
                    with dpg.plot_axis(dpg.mvYAxis, label=""):
                        dpg.add_line_series(Time,Translation_z,label="")

                with dpg.plot(label="Rotation x"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="")
                    with dpg.plot_axis(dpg.mvYAxis, label="Rotation (radians)"):
                        dpg.add_line_series(Time,Rotation_x,label="")

                with dpg.plot(label="Rotation y"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
                    with dpg.plot_axis(dpg.mvYAxis, label=""):
                        dpg.add_line_series(Time,Rotation_y,label="")

                with dpg.plot(label="Rotation z"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="")
                    with dpg.plot_axis(dpg.mvYAxis, label=""):
                        dpg.add_line_series(Time,Rotation_z,label="")

            _add_config_options(subplot_id,1,"link_all_x","link_all_y",before=subplot_id)

        dpg.create_viewport(title='Custom Title', width=800, height=600)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window",True)
        dpg.start_dearpygui()
        dpg.destroy_context()
                

        

if __name__ == '__main__':
    main()
