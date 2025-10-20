import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData, AudioInfo
import numpy as np
import sounddevice as sd
# Importazione specifica per la scrittura di file .wav
from scipy.io.wavfile import write as wavfile_write 

class AudioSubscriber(Node):
    def __init__(self):
        super().__init__('audio_subscriber')

        # 1. LETTURA DEI PARAMETRI CRUCIALI (Definiti nel params.yaml)
        
        # Legge il numero di canali (cruciale per dimensionare i buffer)
        self.declare_parameter('channels', 2)
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        
        # Legge la sample rate
        self.declare_parameter('sample_rate', 44100)
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value

        # Legge l'indice del dispositivo di output per sounddevice.play()
        self.declare_parameter('output_device_index', -1)  
        self.output_device_index = self.get_parameter('output_device_index').get_parameter_value().integer_value 
        
        # Legge la lista dei nomi dei file di output
        self.declare_parameter('output_files', ["mic_01.wav", "mic_02.wav"])
        self.output_files = self.get_parameter('output_files').get_parameter_value().string_array_value
        
        # Legge le configurazioni di topic e riproduzione
        self.declare_parameter('playback_enabled', True)
        self.playback_enabled = self.get_parameter('playback_enabled').value
        
        self.declare_parameter('subscribe_audio_topic', 'audio_stream')
        self.audio_topic = self.get_parameter('subscribe_audio_topic').value
        
        self.declare_parameter('subscribe_info_topic', 'audio_info')
        self.info_topic = self.get_parameter('subscribe_info_topic').value

        # 2. INIZIALIZZAZIONE DEI BUFFER
        # Crea una lista di liste, una lista (buffer) per ogni canale
        self.buffers = [[] for _ in range(self.channels)] 
        self.get_logger().info(f"💾 Pronto a registrare {self.channels} canali separati: {self.output_files}")

        # 3. Sottoscrizione
        # Sottoscrizione alle informazioni (per aggiornamenti runtime, sebbene statiche)
        self.create_subscription(AudioInfo, self.info_topic, self.info_callback, 10)
        # Sottoscrizione ai dati audio
        self.create_subscription(AudioData, self.audio_topic, self.audio_callback, 10)

        self.get_logger().info("🔊 Subscriber pronto a ricevere audio")


    # Aggiorna sample_rate e channels se il Publisher invia un nuovo AudioInfo (sebbene raro)
    def info_callback(self, msg: AudioInfo):
        self.sample_rate = msg.sample_rate
        # NOTA: non aggiorniamo self.channels qui per evitare di corrompere self.buffers,
        # assumendo che il numero di canali sia fisso per la sessione.

    def audio_callback(self, msg: AudioData):
        # Conversione da byte a NumPy array bidimensionale: [campioni, canali]
        data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, self.channels)
        
        # Separazione dei canali e salvataggio nei buffer
        for i in range(self.channels):
            # Prende la colonna 'i' (ovvero tutti i campioni del canale 'i')
            channel_data = data[:, i]  
            self.buffers[i].append(channel_data)  # Aggiunge al buffer dedicato
            
        # Riproduzione audio (manteniamo l'audio multicanale per la riproduzione)
        if self.playback_enabled:
            # Riproduce l'array intero con tutti i canali
            sd.play(data, samplerate=self.sample_rate, device=self.output_device_index)

    def destroy_node(self):
        # Metodo chiamato automaticamente alla chiusura (e da KeyboardInterrupt)
        self.get_logger().info(f"⏹️ Interruzione ricevuta, salvataggio in corso di {self.channels} canali...")
        
        # Salva ogni buffer in un file separato
        for i in range(self.channels):
            if i < len(self.output_files):
                output_filename = self.output_files[i]
                
                if self.buffers[i]:
                    # Concatena tutti i blocchi del canale i
                    recording = np.concatenate(self.buffers[i]) 
                    
                    # Salva il file usando la funzione importata come wavfile_write
                    wavfile_write(
                        output_filename,
                        self.sample_rate,
                        recording
                    )
                    self.get_logger().info(f"💾 Canale {i+1} salvato in '{output_filename}' ({recording.shape[0]} campioni)")
                else:
                    self.get_logger().warn(f"⚠️ Nessun dato ricevuto per il Canale {i+1} ({output_filename})")
            else:
                self.get_logger().warn(f"⚠️ Nome del file mancante per il Canale {i+1}. Ignorato.")
        
        super().destroy_node()
        

def main(args=None):
    rclpy.init(args=args)
    node = AudioSubscriber()
    try:
        # Blocca il nodo e lo mantiene attivo
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gestisce l'interruzione pulita
        pass
    finally:
        # destroy_node gestisce la pulizia e il salvataggio dei file
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

