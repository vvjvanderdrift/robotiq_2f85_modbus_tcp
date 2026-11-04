#!/usr/bin/env python3
"""
Unit tests for Robotiq2F85Gripper class.
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
from robotiq_2f85_modbus_tcp import (
    Robotiq2F85Gripper,
    GripperStatus,
    GripperCommand,
    ActivationState,
    ObjectDetection,
    ConnectionException,
    ActivationException,
    InvalidParameterException,
)


class TestGripperCommand(unittest.TestCase):
    """Test GripperCommand class."""
    
    def test_default_command(self):
        """Test default command values."""
        cmd = GripperCommand()
        self.assertEqual(cmd.r_act, 0)
        self.assertEqual(cmd.r_gto, 0)
        self.assertEqual(cmd.r_atr, 0)
        self.assertEqual(cmd.r_ard, 0)
        self.assertEqual(cmd.r_pr, 0)
        self.assertEqual(cmd.r_sp, 0)
        self.assertEqual(cmd.r_fr, 0)
    
    def test_to_bytes(self):
        """Test command to bytes conversion."""
        cmd = GripperCommand(
            r_act=1,
            r_gto=1,
            r_atr=0,
            r_ard=0,
            r_pr=100,
            r_sp=50,
            r_fr=25
        )
        bytes_data = cmd.to_bytes()
        
        # Check length
        self.assertEqual(len(bytes_data), 16)
        
        # Check action byte (byte 0)
        # rACT=1 (bit 0), rGTO=1 (bit 3)
        # 0b0001001 = 0x11
        self.assertEqual(bytes_data[0], 0b0001001)
        
        # Check reserved bytes
        self.assertEqual(bytes_data[1], 0)
        self.assertEqual(bytes_data[2], 0)
        
        # Check position, speed, force
        self.assertEqual(bytes_data[3], 100)
        self.assertEqual(bytes_data[4], 50)
        self.assertEqual(bytes_data[5], 25)
        
        # Check padding
        for i in range(6, 16):
            self.assertEqual(bytes_data[i], 0)


class TestGripperStatus(unittest.TestCase):
    """Test GripperStatus class."""
    
    def test_default_status(self):
        """Test default status values."""
        status = GripperStatus()
        self.assertEqual(status.g_act, 0)
        self.assertEqual(status.g_gto, 0)
        self.assertEqual(status.g_sta, ActivationState.RESET)
        self.assertEqual(status.g_obj, ObjectDetection.MOVING)
        self.assertEqual(status.g_flt, 0)
        self.assertEqual(status.g_pr, 0)
        self.assertEqual(status.g_po, 0)
        self.assertEqual(status.g_cu, 0)
    
    def test_properties(self):
        """Test status properties."""
        # Test is_activated
        status = GripperStatus(g_act=0)
        self.assertFalse(status.is_activated)
        status.g_act = 1
        self.assertTrue(status.is_activated)
        
        # Test is_moving
        status = GripperStatus(g_gto=0)
        self.assertFalse(status.is_moving)
        status.g_gto = 1
        self.assertTrue(status.is_moving)
        
        # Test has_fault
        status = GripperStatus(g_flt=0)
        self.assertFalse(status.has_fault)
        status.g_flt = 0x05
        self.assertTrue(status.has_fault)
        
        # Test object_detected
        status = GripperStatus(g_obj=ObjectDetection.MOVING)
        self.assertFalse(status.object_detected)
        status.g_obj = ObjectDetection.DETECTED_WHILE_OPENING
        self.assertTrue(status.object_detected)
        status.g_obj = ObjectDetection.DETECTED_WHILE_CLOSING
        self.assertTrue(status.object_detected)
        status.g_obj = ObjectDetection.AT_REQUESTED_POSITION
        self.assertFalse(status.object_detected)
        
        # Test motion_complete
        status = GripperStatus(g_gto=1, g_obj=ObjectDetection.MOVING)
        self.assertFalse(status.motion_complete)
        status = GripperStatus(g_gto=0, g_obj=ObjectDetection.AT_REQUESTED_POSITION)
        self.assertTrue(status.motion_complete)


class TestRobotiq2F85Gripper(unittest.TestCase):
    """Test Robotiq2F85Gripper class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.host = "192.168.1.100"
        self.port = 502
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_connect(self, mock_transport):
        """Test connect method."""
        # Create mock transport
        mock_transport_instance = Mock()
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper(self.host, self.port)
        gripper.connect()
        
        # Verify transport was created and connected
        mock_transport.assert_called_once_with(
            host=self.host,
            port=self.port,
            slave_id=None,
            timeout=10.0
        )
        mock_transport_instance.connect.assert_called_once()
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_disconnect(self, mock_transport):
        """Test disconnect method."""
        mock_transport_instance = Mock()
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper(self.host, self.port)
        gripper.connect()
        gripper.disconnect()
        
        mock_transport_instance.disconnect.assert_called_once()
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_is_connected(self, mock_transport):
        """Test is_connected method."""
        mock_transport_instance = Mock()
        mock_transport_instance.is_connected.return_value = True
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper(self.host, self.port)
        gripper.connect()
        
        self.assertTrue(gripper.is_connected())
        
        mock_transport_instance.is_connected.return_value = False
        self.assertFalse(gripper.is_connected())
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_context_manager(self, mock_transport):
        """Test context manager support."""
        mock_transport_instance = Mock()
        mock_transport.return_value = mock_transport_instance
        
        with Robotiq2F85Gripper(self.host, self.port) as gripper:
            self.assertTrue(gripper.is_connected())
        
        mock_transport_instance.disconnect.assert_called_once()
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_invalid_position(self, mock_transport):
        """Test invalid position parameter."""
        mock_transport_instance = Mock()
        mock_transport_instance.is_connected.return_value = True
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper(self.host, self.port)
        gripper.connect()
        
        # Test position below minimum
        with self.assertRaises(InvalidParameterException):
            gripper.goto_position(-1)
        
        # Test position above maximum
        with self.assertRaises(InvalidParameterException):
            gripper.goto_position(256)
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_invalid_speed(self, mock_transport):
        """Test invalid speed parameter."""
        mock_transport_instance = Mock()
        mock_transport_instance.is_connected.return_value = True
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper(self.host, self.port)
        gripper.connect()
        
        # Test speed below minimum
        with self.assertRaises(InvalidParameterException):
            gripper.goto_position(100, speed=-1)
        
        # Test speed above maximum
        with self.assertRaises(InvalidParameterException):
            gripper.goto_position(100, speed=256)
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_invalid_force(self, mock_transport):
        """Test invalid force parameter."""
        mock_transport_instance = Mock()
        mock_transport_instance.is_connected.return_value = True
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper(self.host, self.port)
        gripper.connect()
        
        # Test force below minimum
        with self.assertRaises(InvalidParameterException):
            gripper.goto_position(100, force=-1)
        
        # Test force above maximum
        with self.assertRaises(InvalidParameterException):
            gripper.goto_position(100, force=256)


class TestGripperActivation(unittest.TestCase):
    """Test gripper activation methods."""
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    @patch('robotiq_2f85_modbus_tcp.gripper.time.sleep')
    def test_activate(self, mock_sleep, mock_transport):
        """Test activate method."""
        # Create mock transport
        mock_transport_instance = Mock()
        mock_transport_instance.is_connected.return_value = True
        
        # Mock status bytes for activation complete
        # Byte 0: gACT=1, gGTO=0, gSTA=3 (Complete)
        # Byte 2: gFLT=0
        # Byte 3: gPR=0
        # Byte 4: gPO=0
        # Byte 5: gCU=0
        mock_transport_instance.read_status_registers.return_value = [
            0b00110001,  # gACT=1, gSTA=3 (Complete)
            0,
            0,  # gFLT=0
            0,  # gPR
            0,  # gPO
            0   # gCU
        ]
        
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper("192.168.1.100")
        gripper.connect()
        
        # Mock get_status to return complete status
        with patch.object(gripper, 'get_status') as mock_get_status:
            mock_get_status.return_value = GripperStatus(
                g_act=1,
                g_gto=0,
                g_sta=ActivationState.COMPLETE,
                g_obj=ObjectDetection.MOVING,
                g_flt=0,
                g_pr=0,
                g_po=0,
                g_cu=0
            )
            
            gripper.activate()
            
            # Verify activation command was sent
            mock_transport_instance.write_command_registers.assert_called()
    
    @patch('robotiq_2f85_modbus_tcp.gripper.ModbusTcpTransport')
    def test_activate_fault(self, mock_transport):
        """Test activation with fault."""
        mock_transport_instance = Mock()
        mock_transport_instance.is_connected.return_value = True
        mock_transport.return_value = mock_transport_instance
        
        gripper = Robotiq2F85Gripper("192.168.1.100")
        gripper.connect()
        
        # Mock get_status to return fault
        with patch.object(gripper, 'get_status') as mock_get_status:
            mock_get_status.return_value = GripperStatus(
                g_act=1,
                g_gto=0,
                g_sta=ActivationState.IN_PROGRESS,
                g_obj=ObjectDetection.MOVING,
                g_flt=0x05,  # Fault
                g_pr=0,
                g_po=0,
                g_cu=0
            )
            
            with self.assertRaises(ActivationException):
                gripper.activate(timeout=0.1)


if __name__ == '__main__':
    unittest.main()
