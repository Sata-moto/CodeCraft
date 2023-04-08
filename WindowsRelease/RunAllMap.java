

import java.util.*;
import java.io.*;

public class RunAllMap {
    public static void main(String[] args) throws Exception {
        int[] scores=new int[4];
        for(int i=0;i<4;i++){
            Process proc = Runtime.getRuntime().exec(String.format("C:/Users/CQH/Desktop/华为软挑/WindowsRelease/Robot -f -m C:/Users/CQH/Desktop/华为软挑/WindowsRelease/maps2/%d.txt -c  \"C:/Users/CQH/Desktop/华为软挑/code/java/src\" \"java com.huawei.codecraft.Main\"",i+1 ));
            //等待进程
            proc.waitFor();
            //读取python返回的json
            BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(proc.getInputStream()));
            String line = null;
            StringBuffer jsonString = new StringBuffer();
            while ((line = bufferedReader.readLine()) != null) {
                System.out.println(line);
                jsonString.append(line);
            }
            bufferedReader = new BufferedReader(new InputStreamReader(proc.getErrorStream()));
            line = null;
            StringBuffer jsonStringErr = new StringBuffer();
            while ((line = bufferedReader.readLine()) != null) {
                System.out.println(line);
                jsonStringErr.append(line);
            }
            
            String str=jsonString.toString();
            String strErr=jsonStringErr.toString();
            // System.out.println(str);
            int j=str.lastIndexOf(':');
            str=str.substring(j+1, str.length());
            str=str.replace("}", "");
            // System.out.println(str);
            int score=Integer.parseInt(str);
            System.out.println(score);
            scores[i]=score;

        }
        int sum=0;
        for(int i=0;i<4;i++){
            sum+=scores[i];
            System.out.printf("第%d幅地图的得分为 %d\n",i+1,scores[i]);
        }
        System.out.printf("总得分为 %d\n",sum);
        
    }
}