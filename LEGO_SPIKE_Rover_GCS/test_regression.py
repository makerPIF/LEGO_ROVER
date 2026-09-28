"""Run: python test_regression.py. Hardware-independent regression tests."""
import ast
import asyncio
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).parent
class Signal:
    def __init__(self, *args): self.values = []
    def emit(self, *args): self.values.append(args)
class QObject: pass

def load_gcs():
    tree = ast.parse((ROOT / 'rover_gcs.py').read_text())
    allowed = []
    for node in tree.body:
        if isinstance(node, (ast.Assign, ast.FunctionDef)):
            if isinstance(node, ast.Assign) and node.lineno > 250: continue
            if isinstance(node, ast.Assign) and not all(isinstance(t, ast.Name) and (t.id.isupper()) for t in node.targets):
                continue
            if isinstance(node, ast.FunctionDef) and node.name == 'main': continue
            allowed.append(node)
        elif isinstance(node, ast.ClassDef) and node.name in ('LinkBase', 'BleLink'):
            allowed.append(node)
    env = dict(QObject=QObject, Signal=Signal, time=time, asyncio=asyncio)
    exec(compile(ast.Module(body=allowed, type_ignores=[]), 'gcs_subset', 'exec'), env)
    return env

G = load_gcs()

def bare_ble():
    b = G['BleLink'].__new__(G['BleLink'])
    G['LinkBase'].__init__(b)
    b._frame=bytearray(); b._priority_frame=bytearray(); b._high_priority=False
    b._tunnel_rx_count=0
    return b

class ProtocolTests(unittest.TestCase):
    def test_known_vectors(self):
        self.assertEqual(G['cobs_pack'](b'\x00'), b'\x00\x00\x02')
        self.assertEqual(G['make_tunnel']('Q'), b'\x32\x03\x00Q\r\n')
        self.assertEqual(G['cobs_pack'](b'\x1e\x00\x00'), bytes.fromhex('071d000002'))
    def test_all_byte_roundtrip(self):
        for data in (bytes(range(256)), b'\x00\x01\x02'*100, b'a'*84, b'a'*85, b''):
            self.assertEqual(G['cobs_unpack'](G['cobs_pack'](data)), data)
    def test_fragmented_and_priority_interleaving(self):
        b=bare_ble(); received=[]; b._on_message=received.append
        low=G['cobs_pack'](b'\x21#T,test\x00')
        high=G['cobs_pack'](b'\x20\x00')
        stream=low[:4]+b'\x01'+high+low[4:]
        for byte in stream: b._on_notify(None, bytes([byte]))
        self.assertEqual(received, [b'\x20\x00', b'\x21#T,test\x00'])
    def test_alive_requires_telemetry(self):
        b=G['LinkBase'](); b.connected=True; b.last_rx=time.monotonic()
        self.assertFalse(b.program_alive())
        b._feed_text('#T,bad\n'); self.assertFalse(b.program_alive())
        b._feed_text('#T,'+','.join(['0']*13)+'\n')
        self.assertTrue(b.program_alive()); self.assertFalse(b.command_ready)
        b._feed_text('#I,ack=Q\n'); self.assertTrue(b.command_ready)
    def test_gate_does_not_send_movement_before_ack(self):
        b=G['LinkBase'](); b.connected=True; sent=[]; b._send_impl=lambda x: sent.append(x) or True
        self.assertFalse(b.send('T100,100')); self.assertTrue(b.send('Q'))
        self.assertEqual(sent, ['Q'])
    def test_safety_mode_blocks_q(self):
        b=G['LinkBase'](); b.connected=True; b.safe_mode=True
        self.assertFalse(b.send('Q'))

class HubTests(unittest.TestCase):
    def setUp(self):
        self.now=0; self.calls=[]; self.lines=[]
        motor=SimpleNamespace(BRAKE=1,COAST=0,run=lambda *a,**k:self.calls.append(('run',a)),stop=lambda *a,**k:self.calls.append(('stop',a)))
        pair=SimpleNamespace(PAIR_1=0, pair=lambda *a:self.calls.append(('pair',a)), unpair=lambda *a:self.calls.append(('unpair',a)), move_tank=lambda *a,**k:self.calls.append(('tank',a)))
        clock=SimpleNamespace(ticks_ms=lambda:self.now,ticks_diff=lambda a,b:a-b,ticks_add=lambda a,b:a+b)
        env=dict(ROVER_VERSION="tunnel-v2",reader=SimpleNamespace(mode="tunnel"),time=clock,motor=motor,motor_pair=pair,PORTS=tuple(range(6)),LEFT_PORT_ID=0,RIGHT_PORT_ID=1,INVERT_LEFT=0,INVERT_RIGHT=1,MAX_VELOCITY=500,ACCEL=400,WATCHDOG_MS=800,emit=self.lines.append,tilt=lambda:(0,0,0))
        tree=ast.parse((ROOT/'hub_rover.py').read_text())
        nodes=[n for n in tree.body if (isinstance(n,ast.ClassDef) and n.name=='Rover') or (isinstance(n,ast.FunctionDef) and n.name in ('clamp','wrap180','handle','control_step'))]
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'hub_subset','exec'),env)
        self.env=env; self.rv=env['Rover'](); self.handle=lambda x:env['handle'](x,self.rv)
    def test_watchdog_heading_and_aux_stop(self):
        self.handle('T100,100'); self.rv.hold=True; self.handle('A2,100')
        self.now=801; self.env['control_step'](self.rv)
        self.assertFalse(self.rv.hold); self.assertEqual(self.rv.cmd_left,0)
        self.assertEqual(self.rv.aux_active,{})
        self.assertTrue({0,1,2}.issubset({a[0] for op,a in self.calls if op=='stop'}))
    def test_q_not_refreshing_drive_watchdog(self):
        self.handle('T100,100'); self.now=700; self.handle('Q'); self.now=801
        self.env['control_step'](self.rv); self.assertEqual(self.rv.cmd_left,0)
        self.assertIn('#I,ack=Q',self.lines)
    def test_estop_zero_keepalive_remains_stopped(self):
        self.handle('E'); self.handle('T0,0'); self.assertTrue(self.rv.estop)
        self.calls.clear(); self.env['control_step'](self.rv); self.assertEqual(self.calls,[])
    def test_ports_validated_and_old_ports_stopped(self):
        self.handle('P2,2'); self.assertEqual((self.rv.left_id,self.rv.right_id),(0,1))
        self.handle('P2,3'); self.assertEqual((self.rv.left_id,self.rv.right_id),(2,3))
        self.assertIn(('stop',(0,)),self.calls); self.assertIn(('stop',(1,)),self.calls)
    def test_zero_drive_does_not_repeat_motor_start(self):
        self.rv.drive(0,0); count=len(self.calls); self.rv.drive(0,0)
        self.assertEqual(len(self.calls),count)
        self.assertFalse(any(op=='run' for op,a in self.calls))
    def test_motor_exception_stops_both(self):
        def fail(*a,**k): raise OSError('missing motor')
        self.env['motor'].run=fail; self.rv.drive(100,100)
        self.assertTrue(self.rv.estop)
        self.assertIn(('stop',(0,)),self.calls); self.assertIn(('stop',(1,)),self.calls)

class TunnelTests(unittest.TestCase):
    def setUp(self):
        HubTests.setUp(self)
        self.gcs=bare_ble()
        self.gcs.connected=True
        self.gcs.sig_line=Signal()
        self.gcs.sig_log=Signal()
        self.sent=[]
        def register(fn): self.callback=fn
        def send(payload):
            self.sent.append(payload)
            packet=bytes([0x32])+len(payload).to_bytes(2, 'little')+payload
            frame=G['cobs_pack'](packet)
            for offset in range(0,len(frame),7):
                self.gcs._on_notify(None,frame[offset:offset+7])
        tunnel=SimpleNamespace(callback=register,send=send)
        self.env.update(_tunnel=tunnel, select=None)
        tree=ast.parse((ROOT/'hub_rover.py').read_text())
        nodes=[n for n in tree.body if (isinstance(n,ast.ClassDef) and n.name=='CommandReader') or (isinstance(n,ast.FunctionDef) and n.name=='emit')]
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'hub_tunnel', 'exec'),self.env)
        self.reader=self.env['CommandReader']()
        self.env['reader']=self.reader

    def deliver(self, text):
        msg=G['make_tunnel'](text)
        self.callback(msg[3:])
        for line in self.reader.pump(): self.handle(line)

    def test_q_roundtrip_through_real_packet_parser(self):
        self.assertEqual(self.reader.mode,'tunnel')
        self.deliver('Q')
        self.assertTrue(self.gcs.command_ready)
        self.assertGreater(self.gcs._tunnel_rx_count,0)
        self.assertEqual(self.reader.rx_bytes,3)

    def test_drive_and_stop_through_callback(self):
        self.deliver('T100,150')
        self.env['control_step'](self.rv)
        self.assertIn(('run',(0,100)),self.calls)
        self.assertIn(('run',(1,-150)),self.calls)
        self.deliver('E')
        self.assertTrue(self.rv.estop)
        self.assertIn(('stop',(0,)),self.calls)

    def test_callback_does_not_drive_directly(self):
        self.callback(b'T100,100\r\n')
        self.assertEqual(self.calls,[])
        self.assertEqual(self.rv.cmd_left,0)

    def test_malformed_tunnel_length_does_not_ack(self):
        self.gcs._on_message(b'\x32\xff\xff#I,ack=Q\n')
        self.assertFalse(self.gcs.command_ready)

    def test_telemetry_uses_tunnel(self):
        self.env['emit']('#T,'+','.join(['0']*13))
        self.assertTrue(self.gcs.program_alive())

    def test_queue_overflow_estop(self):
        for _ in range(17): self.callback(b'T100,100\n')
        self.assertEqual(self.reader.pump(),['E'])
        self.assertEqual(self.reader.pending,[])

    def test_partial_lines(self):
        self.callback(b'T100,'); self.assertEqual(self.reader.pump(),[])
        self.callback(b'200\r\n'); self.assertEqual(self.reader.pump(),['T100,200'])

    def test_console_and_tunnel_buffers_do_not_mix(self):
        self.gcs._feed_text('partial console')
        self.deliver('Q')
        self.assertTrue(self.gcs.command_ready)

if __name__=='__main__': unittest.main(verbosity=2)
