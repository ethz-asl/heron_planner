#!/usr/bin/env python

import rospy
import heron_utils.bt_rendering as render
from flask import Flask, Response
from std_msgs.msg import String

app = Flask(__name__)

# Shared variable to store HTML content
html_content = "<h1>Behaviour tree Loading...</h1>"


@app.route('/')
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Behaviour Tree</title>
        <script>
        function refreshIt(element) {
            setTimeout(function () {
                element.src = element.src.split("?")[0] + "?" + new Date().getTime();
            }, 100);
        }
        </script>
    </head>
    <body>
        <img src="/tree.svg" onload="refreshIt(this)"/>
    </body>
    </html>
    """

@app.route('/tree.svg')
def tree_svg():
    if latest_svg is None:
        return "Cannot recieve HTML from /hlp/html", 500
    
    return Response(latest_svg, mimetype='image/svg+xml')

def svg_cb(msg):
    global latest_svg
    latest_svg = msg.data

rospy.init_node("svg_server", anonymous=True)
rospy.Subscriber("/hlp/html", String, svg_cb)


if __name__=="__main__":
    
    global latest_svg
    latest_svg = ""
    app.run(host="0.0.0.0", port=5050, debug=True, use_reloader=False)
