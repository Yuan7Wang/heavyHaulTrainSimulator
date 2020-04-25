#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  7 11:43:29 2018
@author: xubo
"""
import os
from glob import glob
import numpy as np
import pandas as pd
from pandas.tseries.offsets import *
from datetime import date
from datetime import timedelta
import random
import time


class DataAgent(object):
    def __init__(self, floder_path='./data/test_data_tdx', features_path='./data/all_data_features'):
        print('DataAgent __init__')
        self.floder_path = floder_path
        self.files = [f for f in os.listdir(floder_path) if 'txt' in f]

        self.features_path = features_path
        self.feature_files = [f for f in os.listdir(features_path) if 'txt' in f]

    def data_transform(self, name, feature=False):
#        print('data_transform:', name)
        if not feature:
            path = os.path.join(self.floder_path, name)
            stock_pd = pd.read_table(path, header=None, sep=',', engine='python',
                                     names=['date', 'open', 'high', 'low', 'close', 'volume', 'money'],
                                 parse_dates=['date'],
                                 index_col=0, skiprows=2, skip_blank_lines=True, skipfooter=1)
        else:
            path = os.path.join(self.features_path, name)
#            print('from:', path)
            stock_pd = pd.read_csv(path, parse_dates=['date'], index_col=0, skip_blank_lines=True)
        return stock_pd

    def feature_extract(self, stock):
#        print('feature_extract')
        assert isinstance(stock, pd.core.frame.DataFrame), 'not pandas form'

        # compute basic data
        stock['yes'] = stock['close'].shift(1)  # 获取前一天的收盘价
        stock['ma5']=np.round(pd.Series(stock['close']).rolling(center=False,window=5).mean(), 2)
        stock['ma10']=np.round(pd.Series(stock['close']).rolling(center=False,window=10).mean(), 2)
        stock['ma20']=np.round(pd.Series(stock['close']).rolling(center=False,window=20).mean(), 2)
        stock['ma60']=np.round(pd.Series(stock['close']).rolling(center=False,window=60).mean(), 2)
        stock['v10']=np.round(pd.Series(stock['money']).rolling(center=False,window=10).mean(), 2)
        stock['v20']=np.round(pd.Series(stock['money']).rolling(center=False,window=20).mean(), 2)
        stock['v60']=np.round(pd.Series(stock['money']).rolling(center=False,window=60).mean(), 2)

        # extract features
        # use these feature do cluster or neural network
        stock['o/c'] = (stock['open'] / stock['yes'] - 1) * 100  # open / yesterday close
        stock['h/o'] = (stock['high'] / stock['open'] - 1) * 100  # high / open
        stock['l/o'] = (stock['low'] / stock['open'] - 1) * 100  # low / open
        stock['c/c'] = (stock['close'] / stock['yes'] - 1) * 100  # close / yesterday close
        stock['amp'] = (stock['high'] - stock['low']) / stock['open'] * 100  # amplitude : high - low / open
        stock['c/m10'] = (stock['close'] / stock['ma10'] - 1) * 100  # close / ma10
        stock['c/m20'] = (stock['close'] / stock['ma20'] - 1) * 100  # close / ma20
        stock['c/m60'] = (stock['close'] / stock['ma60'] - 1) * 100  # close / ma60
        stock['v/v10'] = (stock['money'] / stock['v10'] - 1) * 100  # v / v10
        stock['v/v20'] = (stock['money'] / stock['v20'] - 1) * 100  # v / v20
        stock['v/v60'] = (stock['money'] / stock['v60'] - 1) * 100  # v / v60
        stock = stock.dropna()
        return stock

    def init_all_stocks(self, least_days=20, with_feature=True):
        print('DataAgent init_all_stocks')
        self.stocks = {}
        for name in self.files:
            stock_name = name.split('.')[0].split('#')[-1]
            if name in self.feature_files:
                stock_feature = self.data_transform(name, feature=True)
                self.stocks[stock_name] = stock_feature
            else:
                stock_feature = self.data_transform(name)
                if not stock_feature.empty:
                    if with_feature:
                        stock_feature = self.feature_extract(stock_feature)
                        fpath = os.path.join(self.features_path, name)
                        stock_feature.to_csv(fpath, index_label='date', sep=',')
                    if len(stock_feature) >= least_days:
                        self.stocks[stock_name] = stock_feature

    def get_stocks(self, num=300, rand=True, start=0):
        stocks = {}
        if rand:
            names = random.sample(self.files, num)
        else:
            names = self.files[start:start+num]
        for name in names:
            stock_name = name.split('.')[0].split('#')[-1]
            if name in self.feature_files:
                stock_feature = self.data_transform(name, feature=True)
            else:
                stock_feature = self.data_transform(name)
                stock_feature = self.feature_extract(stock_feature)
            if not stock_feature.empty:
                stocks[stock_name] = stock_feature
        return stocks


    def get_stocks_by_name(self, names):
        stocks = {}
        for name in names:
            stocks[name] = self.stocks[name]
        return stocks

    def generate_RR_data(self, start_date, end_date=None, threshold=5, file_name='rr_data.txt'):
        print('generate_RR_data')
        y, m, d = start_date.split('-')
        sd = date(int(y), int(m), int(d))
        ed = date(int(y), int(m), int(d))
        if end_date is not None:
            y, m, d = end_date.split('-')
            ed = date(int(y), int(m), int(d))
        rr_data = {}
        while sd <= ed:
#            print(sd)
            today = []
            for name in self.stocks:
                stock = self.stocks[name]
                if sd in stock.index:
                    zf = stock.loc[sd]['c/c']
                    if zf > threshold:
                        today.append((name, zf))
            rr_data[sd.strftime('%Y%m%d')] = today
            sd += timedelta(days=1)

        with open(file_name,'w') as file:
            for key in sorted(rr_data.keys()):
                s = [temp[0] for temp in rr_data[key]]
                file.write(key + '\t' + ','.join(s)+'\n')
            file.close()
        return rr_data

    def get_random_data(self, num=6):
        print('get_random_data')
        keys = random.sample(list(self.stocks.keys()), num)
        res = {key: value for key, value in self.stocks.items() if key in keys}
        return res

    def get_price(self, name, date):
        print('get_price', name, date)
        if name not in self.stocks or date not in self.stocks[name].index:
            return None
        stock = self.stocks[name].loc[date][['open','high','low','close','volume','money','c/c']]
        return stock



if __name__ == '__main__':
    # data_agent = DataAgent()
    data_agent = DataAgent('data/all_data_tdx')
#    stock_pd = data_agent.data_transform(data_agent.files[-1])
#    stock_feature = data_agent.feature_extract(stock_pd)
    print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
    data_agent.init_all_stocks()
    print(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
#    random_stocks = data_agent.get_random_data(2)
#    print(random_stocks.keys())
#    rr_data = data_agent.generate_RR_data('2000-01-01', '2017-12-29', threshold=5, file_name='up_2000_2017.txt')
#    rr_data = data_agent.generate_RR_data('2016-01-01', '2016-12-29', threshold=5, file_name='up_2016.txt')
#    print(rr_data)
    print(data_agent.get_price('300027', '2017-12-07'))



