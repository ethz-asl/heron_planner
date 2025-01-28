#!/usr/bin/env python

import rospy
import heron_planner.utils.bt_rendering as render
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
    global tree

    if tree is None is tree.root is None:
        return "Tree not initialised", 500
    
    # generate svg automatically
    graph = render.dot_graph(tree.root, include_status=True)
    svg_output = graph.create_svg()
    return Response(svg_output, mimetype='image/svg+xml')


def start_flask_server(tree_instance):
    """run flask server."""
    global tree

    tree = tree_instance
    app.run(host="0.0.0.0", port=5050, debug=True, use_reloader=False)
