import pytest
import zmq
import time
import subprocess
import sys
import os

# Start zmqshell in a subprocess for testing
@pytest.fixture(scope="module")
def zmq_server():
    """Start zmqshell server and return endpoint"""
    # Import the zmqshell module to get its path
    zmqshell_path = os.path.join(os.path.dirname(__file__), "..", "tools", "zmqshell.py")
    
    # Start zmqshell process
    proc = subprocess.Popen(
        [sys.executable, zmqshell_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    time.sleep(1)  # Allow server to start
    
    yield "tcp://127.0.0.1:5555"
    
    proc.terminate()
    proc.wait(timeout=5)


@pytest.mark.parametrize("payload", [
    b"ffmpeg -version",  # Unauthenticated command execution attempt
    b"",  # Empty payload (no auth token)
    b"invalid_token|ffmpeg -h",  # Malformed token format
    b"Bearer expired_token_xyz",  # Expired/invalid bearer token
])
def test_zmqshell_rejects_unauthenticated_requests(zmq_server, payload):
    """Invariant: ZMQ socket must reject all unauthenticated command requests"""
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    
    try:
        socket.connect(zmq_server)
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        
        # Send unauthenticated payload
        socket.send(payload)
        
        # Attempt to receive response
        try:
            response = socket.recv()
            # If we get here without auth, the endpoint is vulnerable
            assert False, f"Unauthenticated request was accepted: {response}"
        except zmq.error.Again:
            # Timeout is acceptable (server rejected silently)
            pass
    finally:
        socket.close()
        context.term()