#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Security Check.

File name: security_check.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import pandas as pd
import time


class SecurityCheck(object):
    """
    Performing the security check for Message Delivery in the State Machine

    Keyword arguments:
    """

    def __init__(self, receiver, recog_elements):
        super(SecurityCheck, self).__init__()
        self.df = pd.DataFrame()
        self.df['type'] = pd.Series(recog_elements.type)
        self.df['label'] = recog_elements.label
        self.df['real'] = recog_elements.real
        self.df['camera'] = recog_elements.camera
        self.receiver = receiver
        self.candidate = pd.DataFrame()

    def can_see_person(self):
        """
        Check if a person is visible

        Keyword arguments:
        """
        if not self.df.empty:
            self.candidate = self.df[(self.df['type'] == 'face') & (self.df['label'].str.lower() == self.receiver.lower())]
        return not self.candidate.empty

    def person_is_real(self):
        """
        Check if a person is real

        Keyword arguments:
        """

        return self.candidate[self.candidate['camera'] == 1]['real'].iloc[0]

    def id_check(self):
        """
        Check if an imperial college ID is visible

        Keyword arguments:
        """
        time.sleep(3)
        id = self.df[(self.df['type'] == 'particular') & (self.df['label'] == 'Imperial College London ID')]
        return not id.empty
