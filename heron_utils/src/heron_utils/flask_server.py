#!/usr/bin/env python

import os
import rospy
import heron_planner.utils.bt_rendering as render
from flask import Flask, render_template_string, jsonify, request
from std_msgs.msg import String
from std_srvs.srv import Trigger

app = Flask(__name__)

SVG_DIRECTORY = "bt_svgs"
os.makedirs(SVG_DIRECTORY, exist_ok=True)
current_status = {"state" : "Stopped"}

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Behaviour Tree Dashboard</title>
    <style>
        .status {
            font-size: 1.5em;
            margin-bottom: 1em;
        }
        .buttons button {
            font-size: 1em;
            margin: 0.5em;
            padding: 0.5em 1em;
            border: none;
            border-radius: 5px;
            cursor: pointer;
        }
        .buttons .start { background-color: green; color: white; }
        .buttons .pause { background-color: orange; color: white; }
        .buttons .stop { background-color: red; color: white; }
    </style>
</head>
<body>
    <h1>Behaviour Tree Dashboard</h1>
    <div class="status">
        <h3> Current Behaviour Tree</h3>
        <pre id="status"></pre>
    </div>
    <div class="buttons">
        <button class="start" onclick="sendCommand('start')">Start</button>
        <button class="pause" onclick="sendCommand('pause')">Pause</button>
        <button class="stop" onclick="sendCommand('stop')">Stop</button>
    </div>
    <div>
        {% if svg_path %}
            <img src="{{ svg_path }}" alt="Behaviour Tree">
        {% else %}
            <p>No SVG found</p>
        {% endif %}
    </div>
    <script>
        function sendCommand(command) {
            fetch('/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command })
            })
            .then(response => response.json())
            .then(data => {
                location.reload(); // Refresh the page to update status
            })
            .catch(error => console.error('Error:', error));
        }
        function updateStatus(data){
            document.getElementById("status").textContent = data.tree || "Tree not avaliable";
        }
    </script>
</body>
</html>
"""


@app.route("/")
def index():
    latest_svg = None
    svg_files = sorted(os.listdir(SVG_DIRECTORY))
    if svg_files:
        latest_svg = os.path.join(SVG_DIRECTORY, svg_files[-1])
    return render_template_string(HTML_TEMPLATE, 
                                  svg_path=latest_svg, 
                                  current_status=current_status["state"])

@app.route("/update_svg", methods=["POST"])
def update_svg():
    """receive an SVG string from the behaviour tree code and save it."""
    svg_content = request.data.decode("utf-8")
    svg_path = os.path.join(SVG_DIRECTORY, "bt_latest.svg")
    with open(svg_path, "w") as f:
        f.write(svg_content)
    return jsonify({"status": "success", "path": svg_path})

@app.route("/update_status", methods=["POST"])
def update_status():
    """Update the current behaviour and state."""
    global current_status
    data = request.json
    current_status["state"] = data.get("state", "Stopped")
    return jsonify({"status": "success", "current_status": current_status})

@app.route("/command", methods=["POST"])
def command():
    """Handle commands from the web interface."""
    data = request.json
    command = data.get("command")
    print(f"Received command: {command}")
    # Logic to handle commands can be implemented here
    if command == "start":
        current_status["state"] = "Started"
    elif command == "pause":
        current_status["state"] = "Paused"
    elif command == "stop":
        current_status["state"] = "Stopped"
    return jsonify({"status": "success", "command": command})


def start_flask_server():
    rospy.init_node("flask_bt_server", anonymous=True)
    rospy.loginfo("starting flask server...")

    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)


if __name__ == "__main__":
    try:
        start_flask_server()
    except rospy.ROSInterruptException:
        pass
