#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String
import xml.etree.ElementTree as ET
import pandas as pd

class GraphDataPublisher:
    def __init__(self):
        rospy.init_node('graph_data_publisher')

        # Parametreleri al
        self.file_path = rospy.get_param('~file_path', '/home/baha/kodcalismalari/boschmap/Competition_track_graph.graphml')

        # Publisher
        self.data_pub = rospy.Publisher('~graph_data', String, queue_size=10)

        # Veri işleme ve yayınlama
        self.process_and_publish_data()

    def process_and_publish_data(self):
        # GraphML dosyasını okuma
        tree = ET.parse(self.file_path)
        root = tree.getroot()

        # Düğüm verilerini çıkarma
        nodes_data = self.extract_nodes_data(root)
        
        # Kenar verilerini çıkarma
        edges_data, edges_data_true = self.extract_edges_data(root)
        
        # Verileri yayınlamak için hazırla
        data_message = String()
        data_message.data = str(edges_data_true)  # Örnek olarak edges_data_true kullanıldı
        self.data_pub.publish(data_message)
        
        rospy.loginfo("Graph data published")

    def extract_nodes_data(self, root):
        nodes_data = []
        for node in root.findall(".//{http://graphml.graphdrawing.org/xmlns}node"):
            node_id = node.get('id')
            d0 = None
            d1 = None
            for data in node:
                if data.get('key') == 'd0':
                    d0 = float(data.text)  # 'x' koordinatı
                elif data.get('key') == 'd1':
                    d1 = float(data.text)  # 'y' koordinatı
            if d0 is not None and d1 is not None:
                nodes_data.append((node_id, d0, d1))
        return nodes_data

    def extract_edges_data(self, root):
        edges_data = []
        edges_data_true = []
        for edge in root.findall(".//{http://graphml.graphdrawing.org/xmlns}edge"):
            source_id = edge.get('source')
            target_id = edge.get('target')
            data_d2 = edge.find(".//{http://graphml.graphdrawing.org/xmlns}data[@key='d2']")
            d2_value = data_d2 is not None and data_d2.text == 'True'
            source_id_int = int(source_id)
            target_id_int = int(target_id)
            ranges = [(469, 479), (409, 425), (386, 398), (357, 365), (259, 281)]
            if any(start <= source_id_int <= end for start, end in ranges) and any(start <= target_id_int <= end for start, end in ranges):
                d2_value = True
                edges_data_true.append((source_id, target_id, d2_value))
            edges_data.append((source_id, target_id, d2_value))
        return edges_data, edges_data_true

if __name__ == '__main__':
    try:
        graph_data_publisher = GraphDataPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
