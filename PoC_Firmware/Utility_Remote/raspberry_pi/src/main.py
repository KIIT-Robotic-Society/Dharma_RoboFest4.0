import asyncio
import subprocess
import signal
import sys
from datetime import datetime
import logging
from multiprocessing import Process, Event

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('scheduler.log'),
        logging.StreamHandler()
    ]
)

class I2CScheduler:
    def __init__(self):
        self.current_process = None
        self.gui_process = None
        self.stop_event = Event()
        self.priority_order = [
            ('joystick_i2c.py', 5),    # Run for 5 seconds
            ('i2clora.py', 3),         # Run for 3 seconds
            ('ESP_NOW_Main_2.py', 2)   # Run for 2 seconds
        ]
        self.initialize_logging()

    def initialize_logging(self):
        self.logger = logging.getLogger('I2CScheduler')
        self.logger.setLevel(logging.INFO)

    def start_gui(self):
        try:
            self.logger.info("Starting GUI process")
            self.gui_process = subprocess.Popen([sys.executable, "gui.py"])
        except Exception as e:
            self.logger.error(f"Error starting GUI: {e}")

    def stop_current_process(self):
        if self.current_process and self.current_process.poll() is None:
            try:
                self.logger.info(f"Stopping process: {self.current_process.args}")
                self.current_process.send_signal(signal.SIGTERM)
                self.current_process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.current_process.kill()
            except Exception as e:
                self.logger.error(f"Error stopping process: {e}")

    async def run_i2c_process(self, script_name, duration):
        try:
            self.stop_current_process()
            self.logger.info(f"Starting {script_name} for {duration} seconds")
            
            self.current_process = subprocess.Popen([sys.executable, script_name])
            
            try:
                await asyncio.sleep(duration)
            except asyncio.CancelledError:
                self.logger.info(f"Process {script_name} was cancelled")
                raise
            finally:
                self.stop_current_process()
                
        except Exception as e:
            self.logger.error(f"Error in run_i2c_process for {script_name}: {e}")

    async def scheduler_loop(self):
        while not self.stop_event.is_set():
            for script_name, duration in self.priority_order:
                if self.stop_event.is_set():
                    break
                
                try:
                    await self.run_i2c_process(script_name, duration)
                except asyncio.CancelledError:
                    self.logger.info("Scheduler loop was cancelled")
                    return
                except Exception as e:
                    self.logger.error(f"Error in scheduler loop: {e}")
                    await asyncio.sleep(1)  # Brief pause before continuing

    def cleanup(self):
        self.logger.info("Cleaning up processes")
        self.stop_event.set()
        self.stop_current_process()
        
        if self.gui_process and self.gui_process.poll() is None:
            try:
                self.gui_process.terminate()
                self.gui_process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.gui_process.kill()
            except Exception as e:
                self.logger.error(f"Error cleaning up GUI process: {e}")

def main():
    scheduler = I2CScheduler()
    
    # Start GUI in separate process
    scheduler.start_gui()
    
    try:
        # Run the async scheduler
        loop = asyncio.get_event_loop()
        loop.run_until_complete(scheduler.scheduler_loop())
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    finally:
        scheduler.cleanup()
        loop.close()

if __name__ == "__main__":
    main()