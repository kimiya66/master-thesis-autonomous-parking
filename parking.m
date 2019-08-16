function ratio=parking(path)
         ratio = 100;
        
         I = imread(path.img);
         detector = load('~/matlabR2018b/bin/newDetectors-8thJun/acfDetector.mat');
         [bboxes, scores]=detector.acfDetector.detect(I);
         %detectedImg = insertObjectAnnotation(I,'rectangle',bboxes,scores,'color','r')   
         %check if anything is detected!
          if isempty(scores)
             sprintf('nothing detected!')
          else
              [selectedBboxes,selectedScores] = selectStrongestBbox(bboxes,scores,'RatioType','Union','OverlapThreshold',0.1)
              for i=1: length(selectedScores)-1
                  if i+1 <= length(selectedScores)
                      if selectedScores(i) > 20 && selectedScores(i+1) > 20 && selectedBboxes(i,4) > 40 
                          B1 = selectedBboxes(i,:);
                          B2 = selectedBboxes(i+1,:);
                          distance=abs(B1-B2)
                          if distance(1,1) <= 130 
                             ratio=bboxOverlapRatio(selectedBboxes(i,:), selectedBboxes(i+1, : ))
                          else
                              sprintf('vehicles do not have enough space from each others')
                          end
                      else
                          sprintf('not enough scores to decide!')
                      end
                     
                  else
                      sprintf('all detection been processed')
                      break;
                  end
              end
          end
end
                
                     
                  
              
              
            