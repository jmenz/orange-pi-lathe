#!/usr/bin/env python

import os
import gtk

class QuitDialog(gtk.Dialog):

    def __init__(self, *args, **kwargs):
        super(QuitDialog, self).__init__(*args, **kwargs)

        self.builder = gtk.Builder()
        self.builder.add_from_file(os.path.join(os.path.dirname(__file__),"quit.ui"))
        self.dialog = self.builder.get_object("dialogQuit")
        self.dialog.set_modal(True)
        self.value = -1

        self.builder.get_object("btnShutdown").connect("button-press-event", self.on_button_press,2)
        self.builder.get_object("btnRestart").connect("button-press-event", self.on_button_press,1)
        self.builder.get_object("btnClose").connect("button-press-event", self.on_button_press,0)
        self.builder.get_object("btnCancel").connect("button-press-event", self.on_button_press,-1)

    def on_button_press(self, widget, event, code):
        self.value = code

    def get_value(self):
        return self.value

    def run(self):

        ret = False

        self.dialog.set_modal(True)
        if self.dialog.run() == 1:
            ret = True
        self.dialog.hide()
        return ret
