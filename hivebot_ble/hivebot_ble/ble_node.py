import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib
import threading
import queue

BLUEZ_SERVICE = 'org.bluez'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'

SERVICE_UUID = '12345678-1234-5678-1234-56789abcdef0'
WRITE_UUID = '12345678-1234-5678-1234-56789abcdef1'
NOTIFY_UUID = '12345678-1234-5678-1234-56789abcdef2'


# ??? BLE Advertisement ???

class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/hivebot/advertisement'

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = 'peripheral'
        self.local_name = 'Hivebot'
        self.service_uuids = [SERVICE_UUID]
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            LE_ADVERTISEMENT_IFACE: {
                'Type': self.ad_type,
                'LocalName': dbus.String(self.local_name),
                'ServiceUUIDs': dbus.Array(self.service_uuids, signature='s'),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        return self.get_properties()[interface]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature='', out_signature='')
    def Release(self):
        print('Advertisement released')


# ??? GATT Service ???

class Service(dbus.service.Object):
    PATH_BASE = '/org/bluez/hivebot/service'

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = SERVICE_UUID
        self.primary = True
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                'UUID': self.uuid,
                'Primary': self.primary,
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        return self.get_properties()[interface]


# ??? Write Characteristic (Phone ? Robot) ???

class WriteCharacteristic(dbus.service.Object):
    def __init__(self, bus, index, service, ros_node):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = WRITE_UUID
        self.service = service
        self.flags = ['write', 'write-without-response']
        self.ros_node = ros_node
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
            }
        }

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        return self.get_properties()[interface]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}', out_signature='')
    def WriteValue(self, value, options):
        msg = bytes(value).decode('utf-8').strip()
        self.ros_node.handle_ble_message(msg)


# ??? Notify Characteristic (Robot ? Phone) ???

class NotifyCharacteristic(dbus.service.Object):
    def __init__(self, bus, index, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = NOTIFY_UUID
        self.service = service
        self.flags = ['notify']
        self.notifying = False
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
            }
        }

    @dbus.service.method(DBUS_PROP_IFACE, in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        return self.get_properties()[interface]

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        self.notifying = True

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        self.notifying = False

    def send_notification(self, message):
        if not self.notifying:
            return
        value = [dbus.Byte(b) for b in message.encode('utf-8')]
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {'Value': dbus.Array(value, signature='y')},
            []
        )

    @dbus.service.signal(DBUS_PROP_IFACE, signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


# ??? GATT Application ???

class Application(dbus.service.Object):
    def __init__(self, bus, ros_node):
        self.path = '/org/bluez/hivebot'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

        svc = Service(bus, 0)
        write_char = WriteCharacteristic(bus, 0, svc, ros_node)
        notify_char = NotifyCharacteristic(bus, 1, svc)
        svc.characteristics.append(write_char)
        svc.characteristics.append(notify_char)
        self.services.append(svc)

        ros_node.notify_char = notify_char

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for svc in self.services:
            response[svc.get_path()] = svc.get_properties()
            for chrc in svc.characteristics:
                response[chrc.path] = chrc.get_properties()
        return response


# ??? ROS 2 Node ???

class BLENode(Node):
    def __init__(self):
        super().__init__('hivebot_ble')

        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.booking_pub = self.create_publisher(String, '/ble/booking', qos)
        self.command_pub = self.create_publisher(String, '/ble/commands', qos)
        self.nav_goal_pub = self.create_publisher(String, '/ble/navigation_goal', qos)

        self.status_sub = self.create_subscription(
            String, '/ble/status', self.status_callback, 10
        )

        self.notify_char = None
        self.msg_queue = queue.Queue()
        self.create_timer(0.1, self.process_queue)

        self.get_logger().info('Hivebot BLE node started')
        self.get_logger().info(f'Service UUID: {SERVICE_UUID}')
        self.get_logger().info(f'Write UUID:  {WRITE_UUID}')
        self.get_logger().info(f'Notify UUID: {NOTIFY_UUID}')
        self.get_logger().info('Topics:')
        self.get_logger().info('  /ble/booking          (BLE -> ROS)')
        self.get_logger().info('  /ble/commands          (BLE -> ROS)')
        self.get_logger().info('  /ble/navigation_goal   (BLE -> ROS)')
        self.get_logger().info('  /ble/status            (ROS -> BLE)')

    def handle_ble_message(self, msg):
        """Queue incoming BLE message for processing on ROS thread"""
        self.get_logger().info(f'[BLE] Received: {msg}')
        if msg == 'ping':
            return
        self.msg_queue.put(msg)

    def process_queue(self):
        """Process queued BLE messages on the ROS thread"""
        while not self.msg_queue.empty():
            msg = self.msg_queue.get()
            ros_msg = String()
            ros_msg.data = msg

            if msg.startswith('book:'):
                self.booking_pub.publish(ros_msg)
                self.get_logger().info(f'[BOOKING] {msg}')
                self.send_to_phone('confirmed')

            elif msg.startswith('follow:'):
                self.command_pub.publish(ros_msg)
                self.get_logger().info(f'[COMMAND] {msg}')

            elif msg.startswith('goto:'):
                self.nav_goal_pub.publish(ros_msg)
                self.get_logger().info(f'[NAV GOAL] {msg}')

            elif msg == 'loaded':
                self.command_pub.publish(ros_msg)
                self.get_logger().info(f'[COMMAND] Bags loaded')

            elif msg == 'done':
                self.command_pub.publish(ros_msg)
                self.get_logger().info(f'[COMMAND] Session ended')
                self.send_to_phone('session_ended')

            elif msg == 'cancel':
                self.command_pub.publish(ros_msg)
                self.get_logger().info(f'[COMMAND] Booking cancelled')
                self.send_to_phone('cancelled')

            else:
                self.command_pub.publish(ros_msg)
                self.get_logger().warn(f'[UNKNOWN] {msg}')

    def status_callback(self, msg):
        """Forward ROS status messages to phone via BLE notify"""
        self.get_logger().info(f'[STATUS -> PHONE] {msg.data}')
        self.send_to_phone(msg.data)

    def send_to_phone(self, message):
        """Send a message to the phone via BLE notify characteristic"""
        if self.notify_char:
            try:
                self.notify_char.send_notification(message)
                self.get_logger().info(f'[BLE NOTIFY] Sent: {message}')
            except Exception as e:
                self.get_logger().error(f'[BLE NOTIFY] Failed: {e}')
        else:
            self.get_logger().warn('[BLE NOTIFY] No notify characteristic available')

    def start_ble(self):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SystemBus()
        adapter_obj = bus.get_object(BLUEZ_SERVICE, '/org/bluez/hci0')

        gatt_manager = dbus.Interface(adapter_obj, GATT_MANAGER_IFACE)
        app = Application(bus, self)
        gatt_manager.RegisterApplication(
            app.get_path(), {},
            reply_handler=lambda: self.get_logger().info('GATT application registered'),
            error_handler=lambda e: self.get_logger().error(f'Failed to register app: {e}')
        )

        ad_manager = dbus.Interface(adapter_obj, LE_ADVERTISING_MANAGER_IFACE)
        adv = Advertisement(bus, 0)
        ad_manager.RegisterAdvertisement(
            adv.get_path(), {},
            reply_handler=lambda: self.get_logger().info('Advertisement registered'),
            error_handler=lambda e: self.get_logger().error(f'Failed to register ad: {e}')
        )

        self.mainloop = GLib.MainLoop()
        self.ble_thread = threading.Thread(target=self.mainloop.run, daemon=True)
        self.ble_thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = BLENode()
    node.start_ble()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info('Shutting down BLE node')
    if hasattr(node, 'mainloop'):
        node.mainloop.quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
