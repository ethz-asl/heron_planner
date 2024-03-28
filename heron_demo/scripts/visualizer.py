"""Class that renders the BT in a web browser and allows ticking the tree manually."""

import os
import shutil
import tempfile
import threading
import webbrowser
import sys
import platform

from heron_planner.bt_rendering import dot_graph
import py_trees as pt

class BTVisualizer:
    """Render the BT in a web browser and allows ticking the tree manually."""
   
    if platform.system() == "Linux":
        CHROME_PATH = r"/usr/bin/google-chrome-stable"
    elif platform.system() == "Windows":
        CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
    else:
        raise NotImplementedError("Platform type not implemented, cannot visualize BT")

    DISPLAY_HTML = '<!DOCTYPE html>\
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

    def __init__(self, tree):
        if isinstance(tree, pt.trees.BehaviourTree):
            self.tree = tree.root
        else:
            self.tree = tree
        self.temp_dir = tempfile.mkdtemp()
        self.html_document = os.path.join(self.temp_dir, "behavior_tree.html")
        self.svg_document = os.path.join(self.temp_dir, "tree.svg")

        dot_graph(self.tree, True).write_svg(self.svg_document, encoding="utf8")
        with open(self.html_document, "w") as f:
            f.write(self.DISPLAY_HTML)

        self.__thread = threading.Thread(target=self.__open_browser)
        self.__thread.start()

    def __open_browser(self):
        if not webbrowser.get(f'"{self.CHROME_PATH}" %s').open(
            "file://" + self.html_document
        ):
            webbrowser.open("file://" + self.html_document)

    def __del__(self):
        while os.path.isdir(self.temp_dir):
            try:
                f = open(self.svg_document, encoding="utf8")
                f.close()
                shutil.rmtree(self.temp_dir)
            except IOError:
                pass

    def tick(self) -> pt.common.Status:
        """Tick the tree once and display its status."""
        self.tree.tick_once()
        self.update_graph(self.tree)
        return self.tree.status

    def update_graph(self, bt: pt.trees.BehaviourTree):
        """Update the visualized graph."""
        dot_graph(bt, True).write_svg(self.svg_document, encoding="utf8")
