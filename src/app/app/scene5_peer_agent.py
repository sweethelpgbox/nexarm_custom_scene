#!/usr/bin/env python3
# encoding: utf-8
"""
Scene 5 Peer Agent - B 机开机自启（非 ROS2 节点）
B 机镜像已将 scene_5 / role=B / domain_id 写死在 .typerc，
收到 A 机 UDP 信号后直接 TCP 回连告知"已就绪"，不改配置不重启。
"""
import os
import json
import socket
import threading

A_TCP_PORT    = 9876   # B 连 A 的 TCP 端口
B_SIGNAL_PORT = 9875   # B 监听 A 的 UDP 端口


def _current_state():
    return {
        'status': 'ready',
        'scene': os.environ.get('CALIBRATION_CURRENT_SCENE', 'scene_5'),
        'arm_role': os.environ.get('SCENE5_ARM_ROLE', 'B'),
        'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
    }


def _connect_to_a_and_respond(host):
    """收到 A 的信号后，TCP 连 A，读取请求，回复当前状态，无需修改配置或重启。"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10.0)
        sock.connect((host, A_TCP_PORT))
        sock.settimeout(None)
        print(f'[B] 已连接 A 机 TCP {host}:{A_TCP_PORT}')

        data = b''
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            data += chunk

        if data:
            try:
                msg = json.loads(data.decode())
                print(f'[B] 收到 A 机消息: {msg}')
            except Exception:
                pass

        state = _current_state()
        print(f'[B] 回复 A 机: {state}')
        sock.sendall(json.dumps(state).encode())
        sock.close()

    except Exception as e:
        print(f'[B] 连接 A 失败: {e}')


def _udp_signal_listener():
    """监听 A 发来的 UDP 信号：'connect_request'"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', B_SIGNAL_PORT))
    print(f'[B] Peer Agent 等待 A 机信号 UDP:{B_SIGNAL_PORT} ...')
    while True:
        try:
            data, addr = sock.recvfrom(256)
            msg = data.decode().strip()
            if msg == 'connect_request':
                print(f'[B] 收到 A 机连接请求，回连 {addr[0]} ...')
                threading.Thread(
                    target=_connect_to_a_and_respond,
                    args=(addr[0],),
                    daemon=True,
                ).start()
        except Exception as e:
            print(f'[B] UDP 监听异常: {e}')


def main():
    print('[B] Peer Agent 启动')
    _udp_signal_listener()   # 阻塞


if __name__ == '__main__':
    main()
