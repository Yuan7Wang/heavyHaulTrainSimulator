#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 10 10:56:20 2018

@author: xubo
"""
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import matplotlib.dates as mdate
from matplotlib.font_manager import FontProperties


class VisualizeAgent(object):
    def __init__(self):
        print('VisualizeAgent __init__')

    def draw_trends(self, stocks, days=20, with_volume=False, row_num=3, fill_row=False, last_date=None):
        print('draw_trends')
        assert isinstance(stocks, dict), 'must be dict with name:data structure'
        num = len(stocks)
        column_count = row_num if np.sqrt(num) > row_num else np.ceil(np.sqrt(num))
        if fill_row:
            column_count = row_num
        row_count = np.ceil(num / column_count)
        column_count = int(column_count)
        row_count = int(row_count)
        count = 0
        height = 3.5 if with_volume else 2.5
        fig, axes = plt.subplots(row_count, column_count,figsize=(column_count*4, row_count*height))
        for name, data in stocks.items():
            ax = plt.subplot(row_count, column_count, count+1)
            if last_date:
                query = 'date <= "' + last_date + '"'
                data = data.query(query)
            values = data.tail(days)
            ds = values.describe()
            total_high = ds['high']['max']
            total_low = ds['low']['min']
            date = data[-1:].index.strftime('%Y%m%d')[0]
            money_max = ds['money']['max']

            total_count = len(values)
#            ax.xaxis.set_major_formatter(mdate.DateFormatter('%Y-%m-%d'))#设置时间标签显示格式
            bottom = total_low * 1.1 - total_high * 0.1
            if with_volume:
                bottom = total_low * 1.6 - total_high * 0.6
            plt.axis([0, total_count*7 + 2, bottom, total_high*1.1 - total_low * 0.1])
            for i in range(total_count):
                v = values.iloc[i]
                x = i*7 + 2
                y = v['open']
                h = v['close'] - v['open']
                w = 5
                cl = 'red' if h > 0 else 'green'
                rf = False if h > 0 else True
                rect = mpatches.Rectangle([x, y], w, h, ec="none", color=cl)
                ax.add_patch(rect)
                if with_volume:
                    m = v['money']
                    m = (total_high-total_low) * 0.45 * m / money_max
                    v_rect = mpatches.Rectangle([x, bottom], w-1, m, ec=cl, fill=rf, fc=cl)
                    ax.add_patch(v_rect)
                line = mlines.Line2D([x+2.25,x+2.25], [v['high'],v['low']], color=cl, lw=.5, alpha=1)
                ax.add_line(line)
            ax.text(2, total_high*1.01 - total_low * 0.01, name, fontsize=10)
            mdx = [x*7 for x in range(total_count)]
            md10y = values['ma10'].values
            md5y = values['ma5'].values
            split = total_low * 1.1 - total_high * 0.1
            ax.text(7*total_count*0.78, split, date, fontsize=9)
            if with_volume:
                p_v_split = [split for _ in range(total_count+1)]
                plt.plot(mdx+[total_count*7],p_v_split,color='#AAAAAA', linestyle='dashdot', linewidth=0.5)
#                font = FontProperties(fname=r'/Library/Fonts/Songti.ttc', size=9)
#                ax.text(2, split, str(round(money_max/100000000, 2))+'亿', fontsize=9, fontproperties=font)
                ax.text(2, split, str(round(money_max/100000000, 2))+' (bn)', fontsize=9)
            plt.plot(mdx,md10y,color='#555555', linestyle='dashdot', linewidth=0.5)
            plt.plot(mdx,md5y,color='#5555F5', linestyle='dashdot', linewidth=0.5)
            count += 1
        plt.show()

    def draw_curve(self, money, dates):
        fig, ax = plt.subplots()
        fig.set_size_inches(12, 7)
        # 指定颜色和线型，具体在console中plt.plot? 便能看到所有参数信息
        plt.plot(money, dates, color='#555555', linestyle='dashdot',
                 marker='o', markerfacecolor='r', markersize=5)
        plt.title('Equity Curve', fontsize=15)
        plt.xlabel('day', fontsize=15)
        plt.ylabel('money', fontsize=15)
        plt.show()

import DataAgent
if __name__ == '__main__':
    print('visualize_agent_main')
    data_agent = DataAgent.DataAgent()
#    data_agent.init_all_stock()
    random_stocks = data_agent.get_stocks(4)
    print(random_stocks.keys())
    va = VisualizeAgent()
    va.draw_trends(random_stocks)
    va.draw_trends(random_stocks, days=30, with_volume=True, row_num=4)
    va.draw_trends(random_stocks, days=30, with_volume=True, row_num=4, fill_row=True, last_date='2017-12-11')


