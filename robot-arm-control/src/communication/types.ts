export interface SerialMessage {
  type: 'POS' | 'ENDSTOP' | 'OK' | 'ERROR' | 'HOMED';
  data: any;
  timestamp: number;
}

export interface Command {
  type: 'MOVE' | 'HOME' | 'QUERY' | 'ENABLE' | 'STOP';
  payload: any;
  id: string;
  timestamp: number;
}
