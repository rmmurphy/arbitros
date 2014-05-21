function [ pError ] = phaseError( phase1, phase2 )

   pError = phase1 - phase2;
   
   if( pError <= -pi)
      pError = pError + 2*pi;
   elseif( pError > pi)
      pError = pError - 2*pi;
   end
   
end

