#!/usr/bin/env python
import zerorpc
import sys

def test_robot_server(nuc_ip, port=4242):
    """Test connection to robot server and try to call methods"""
    print(f"Testing robot server at {nuc_ip}:{port}...")
    
    try:
        client = zerorpc.Client(timeout=10)
        client.connect(f"tcp://{nuc_ip}:{port}")
        print("✓ Connected to ZeroRPC server")
        
        # Try to call launch_robot (the method that's failing in your code)
        print("\nAttempting to call launch_robot()...")
        try:
            result = client.launch_robot()
            print(f"✓ launch_robot() succeeded: {result}")
        except zerorpc.exceptions.RemoteError as e:
            print(f"✗ launch_robot() failed with RemoteError:")
            print(f"   {e}")
            return False
        except Exception as e:
            print(f"✗ launch_robot() failed: {type(e).__name__}: {e}")
            return False
            
        client.close()
        return True
        
    except Exception as e:
        print(f"✗ Connection failed: {type(e).__name__}: {e}")
        return False

if __name__ == "__main__":
    NUC_IP = "192.168.1.100"  # Replace with your NUC IP
    
    if len(sys.argv) > 1:
        NUC_IP = sys.argv[1]
    
    success = test_robot_server(NUC_IP)
    sys.exit(0 if success else 1)