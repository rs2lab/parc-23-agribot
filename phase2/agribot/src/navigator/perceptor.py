import rospy


class AgribotAgent:
    def __init__(self, node_id = 'smart_unicv_agribot_navigator_agent', **kwargs) -> None:
        self._node_id = node_id

        debug = kwargs['debug'] if 'debug' in kwargs else False
        log_level = rospy.DEBUG if debug else rospy.INFO

        rospy.init_node(self._node_id, anonymous=False, log_level=log_level)
        rospy.on_shutdown(self._on_shutdown)
        rospy.logdebug(f'-- Node {self._node_id!r} initialized')
        # TODO

    def _on_shutdown(self):
        pass

    # def 