#!/usr/bin/env python

"""Class that renders the BT in a web browser and allows ticking the tree manually."""

import os
import tempfile
import shutil
import threading
import time

from flask import Flask, send_file, jsonify

from bt_rendering import dot_graph
import py_trees as pt


class BTVisualiser:
    """Render the BT in a web browser and allows ticking the tree manually."""


    def __init__(self, tree):

        if isinstance(tree, pt.trees.BehaviourTree):
            self.tree = tree.root
        elif isinstance(tree, pt.behaviour.Behaviour):
            self.tree = tree
        else:
            raise ValueError

        self.temp_dir = tempfile.mkdtemp()
        self.svg_document = os.path.join(self.temp_dir, "tree.svg")

        self.display_html = '<!DOCTYPE html>\
                            <html>\
                            <head>\
                                <meta charset="utf-8" />\
                                <title>Behavior Tree</title>\
                                <script language="JavaScript">\
                                    function refreshIt(element) {\
                                        setTimeout(function () {\
                                            element.src = element.src.split("?")[0] + "?" +\
                                            new Date().getTime();\
                                        }, 100);\
                                    }\
                                </script>\
                            </head>\
                            <body>\
                                <img src="tree.svg" onload="refreshIt(this)"/>\
                            </body>\
                            </html>'
        
        # generate initial tree visualisation
        self.update_graph(self.tree)

        # start flask server in seperate thread
        self.app = Flask(__name__)
        self.setup_routes()
        threading.Thread(target=self.run_server, daemon=True).start()

    def setup_routes(self):
        @self.app.route("/tree.svg")
        def serve_svg():
            return send_file(self.svg_document, mimetype="image/svg+xml")

        @self.app.route("/")
        def index():
            return self.display_html
            # return """<!DOCTYPE html>
            #     <html>
            #     <head>
            #         <meta charset="utf-8" />
            #         <title>Behavior Tree</title>
            #         <script language="JavaScript">
            #             function refreshIt(element) {
            #                 setTimeout(function () {
            #                     element.src = element.src.split("?")[0] + "?" +
            #                         new Date().getTime();
            #                 }, 100);
            #             }
            #         </script>
            #     </head>
            #     <body>
            #         <h1>Behavior Tree Visualizer</h1>
            #         <img src="tree.svg" onload="refreshIt(this)"/>
            #         <script>
            #             function refreshIt(element) {
            #                 setTimeout(function () {
            #                     element.src = element.src.split("?")[0] + "?" + new Date().getTime();
            #                 }, 100); // refresh the image after a tick
            #             }
            #         </script>
            #     </body>
            #     </html>"""

    def run_server(self):
        self.app.run(port=5000, debug=True, use_reloader=False)

    def update_graph(self, bt: pt.trees.BehaviourTree):
        """Update the visualized graph."""
        dot_graph(bt, True).write_svg(self.svg_document, encoding="utf8")

    def __del__(self):
        while os.path.isdir(self.temp_dir):
            try:
                shutil.rmtree(self.temp_dir)
            except Exception as err:
                print(f"Exception raised: {err}")
                pass

