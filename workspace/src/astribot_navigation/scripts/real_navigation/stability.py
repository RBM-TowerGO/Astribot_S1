"""Read-only settling evidence; never authorizes or resumes motion."""
from collections import deque
import math


class Window:
    def __init__(self, gap):
        self.gap=gap;self.rows=deque(maxlen=512)

    def add(self,now,values):
        if not math.isfinite(now) or not all(math.isfinite(v) for v in values):
            self.rows.clear();return
        if self.rows and (now<=self.rows[-1][0] or now-self.rows[-1][0]>self.gap):self.rows.clear()
        self.rows.append((now,list(values)))
        # Keep one sample immediately before the 1 s boundary.
        while len(self.rows)>2 and self.rows[1][0]<=now-1.:self.rows.popleft()

    def ready(self,now):
        return len(self.rows)>=3 and 0<=now-self.rows[-1][0]<self.gap and self.rows[-1][0]-self.rows[0][0]>=1.

    def span(self,now):
        if not self.ready(now):return None
        return [max(row[1][i] for row in self.rows)-min(row[1][i] for row in self.rows) for i in range(len(self.rows[0][1]))]

    def mean(self,now):
        if not self.ready(now):return None
        duration=self.rows[-1][0]-self.rows[0][0]
        return [sum((b[0]-a[0])*b[1][i] for a,b in zip(self.rows,list(self.rows)[1:]))/duration
                for i in range(len(self.rows[0][1]))]
