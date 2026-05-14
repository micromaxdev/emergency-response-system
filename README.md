# emergency-response-system
ERS developed through ECTE351 and further work in house

Project Architecture:
  websites:the streamlit based web 
    web_main.py: The primary entry point for the web application
    utils.py: A global utility toolkit managing database connectivity, path anchoring, and shared CSS
    pages: Multi-page functional modules (Dashboard, Logs, Staff Management, and System Configuration)

  server: Core Backend Logic
    ers.sqlite: The system's central data hub (SQLite Database)
    alert_handlers: Pluggable alert processing modules (Email, SMS, Audio, and Relay control)

  assets: Static Resource Repository
    maps: Storage for office and facility floor plans
    audios: Repository for pre-configured alert audio files

  client: Source code for remote LoRa terminal devices

  venv: Project-specific Python Virtual Environment

  logs:stores the system's running records, such as events and error messgaes

  data:stores the system's long-term structured data,such as device id and staff id



1. Environmental Setup
Clone the repository and navigate to the project root:
cd /home/micromax/emergency-response-system

2. Initialize virtual environment
Activate the dedicated virtual environment and install all necessary dependencies:
source venv/bin/activate
pip install -r requirements.txt

3. Start the backend server(starts listening for incoming LoRa signals from gateways)
python3 server/server.py

4. Start the web interface
python3 -m streamlit run websites/web_main.py