function write on file(post_process_data, par)

  fileID = fopen(par.file_name, 'a');
  fprintf(fileID, '%s\n', post_process_data);
  fclose(fileID);


  prob_GPS 0.1 stx_x std_y std_z
  prob_GPS 0.2 stx_x std_y std_z
  prob_comm 0.1 stx_x std_y std_z
