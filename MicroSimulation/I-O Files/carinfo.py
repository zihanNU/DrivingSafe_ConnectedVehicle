import os
 

class MEASURE:
   
   def __init__(self, carid, acc, decc,dis,risk,time):
        self.carid=carid
        self.acc=acc
        self.decc=decc
        self.dis=dis
        self.risk=risk
   def writeCOUNT(self,output):
       #output1.write(self.cnt+'\n')
       output.write(self.did+','+self.obflag+','+self.time+','+self.cnt+','+self.vph+','+self.occ+','+self.spd+'\n')
       output.close


cardetail=open("caronly.txt",'r')
cars=[]
carsline=cardetail.readlines()
for i in range(1,len(carsline)):
    did=i
    
    
    if did !='0':
        detectorids.append(did)
linkdetail.close()


linkoutput1=open('linkcounts.dat','w')
linkoutput1.write('param OBSFLOW :=\n')
#linkflow=[]
for obsfile in filenames:
   linkobs=gzip.open(newpath+'\\'+obsfile,'r')
   obsline=linkobs.readlines()
for i in range(1,1000):
               #len(obsline)):
    features=obsline[i].rstrip('\n').split(',')
    did=features[0]
    if did in detectorids:
        timestamp=features[4].split(' ')
        date=timestamp[0]
        time=timestamp[1]
        year=date.split('/')[2]
        month=date.split('/')[0]
        day=date.split('/')[1]
        flag=detectorids.index(did)
        linkoutput2=open('obs_'+year+'_'+month+'_'+day+'.csv','a+')
        l=MEASURE(did,features[8],features[9],features[10],features[11],time,date,str(flag))
        #linkflow.append(l)
        l.writeCOUNT(linkoutput2)
linkobs.close()


           
    
