#!/usr/bin/env python
# -*- coding: utf-8 -*-

import urllib, urllib2, json, base64, socket, subprocess

# @class Engine class
class SSEngine(object):
    NICT = 'nict'
    GOOGLE = 'google'

# @class Error Class
class SSError(IOError):
    def __init__(self, reason):
        self.args = reason,
        self.reason = reason

    def __str__(self):
        return '<speech sythesis error> %s' % self.reason


# @class SpeechSynthesisFactory Class
# @brief 音声合成クラスを生成するためのファクトリークラス
class SpeechSynthesisFactory(object):
    # @brief 指定した音声合成エンジンのクライアントを生成する
    # @param[in] engine 音声合成エンジン (nict / google)
    @classmethod
    def create(cls, engine=''):
        if engine == 'google':
            return SpeechSynthesisClient_Google()
        elif engine == 'nict':
            return SpeechSynthesisClient_NICT()
        else:
            raise SSError( 'target client %s is not found.'%engine )

# @class SpeechSynthesisCient Class
# @brief 音声合成をするクライアントのインタフェースクラス
class SpeechSynthesisClient(object):
    # @brief 音声合成のリクエストをサーバに送信し、合成結果を取得する
    # @param[in] message 音声合成する文字列
    # @param[in] language 音声合成する言語
    # @param[in] voice_font ボイスフォントの指定
    # @param[in] timeout タイムアウト時間
    def request(self, message, language='ja', voice_font='', timeout=socket._GLOBAL_DEFAULT_TIMEOUT):
        raise SSError("error")

# @class SpeechSynthesisCient_NICT Class
# @brief 音声合成をするクライアント
class SpeechSynthesisClient_NICT(SpeechSynthesisClient):
    # @brief コンストラクタ
    def __init__(self):
        self.URL = 'http://rospeex.ucri.jgn-x.jp/nauth_json/jsServices/VoiceTraSS'
        self.FORMAT = 'x-wav'

    # @brief 音声合成のリクエストをサーバに送信し、合成結果を取得する
    # @param[in] message 音声合成する文字列
    # @param[in] language 音声合成する言語
    # @param[in] voice_font ボイスフォントの指定
    # @param[in] timeout タイムアウト時間
    # @return 音声合成結果（バイナリデータ）
    def request(self, message, language='ja', voice_font='*', timeout=socket._GLOBAL_DEFAULT_TIMEOUT):
        # create request data
        data = {
            'method':'speak',
            'params':[
                '1.1',
                {'language':language,
                 'text':message,
                 'voiceType':voice_font,
                 'audioType':'audio/%s'%self.FORMAT,
                 'applicationType':'rospeex'
                }
            ]
        }

        # create request
        request = urllib2.Request(self.URL)
        request.add_header('Content-Type', 'application/json')

        # get response
        voice = None
        try:
            response = urllib2.urlopen(request, json.dumps(data))
            decord = json.loads( response.read() )
            voice = base64.b64decode(decord['result']['audio'])
        except urllib2.HTTPError, e:
            raise SSError( 'http error. code:%s msg:%s'%(e.code, e.msg) )
        except urllib2.URLError, e:
            raise SSError( 'request url error. reason:%s'%e.reason )
        except:
            raise SSError('unknown exception.')

        return voice

# @class SpeechSynthesisCient_Google Class
# @brief 音声合成をするクライアント
class SpeechSynthesisClient_Google(SpeechSynthesisClient):
    # @brief コンストラクタ
    def __init__(self):
        self.URL = 'http://translate.google.com/translate_tts?'

    # @brief convert mp3 to wav
    # @param[in] data voice data (mp3)
    # @return voice data (wav)
    def _convert_mp3_to_wav(self, data):
        # convert wav to flac data
        try :
            input_file_name = 'ss_tmp.mp3'
            output_file_name = 'ss_tmp.wav'
            fo = open(input_file_name, 'wb')
            fo.write(data)
            subprocess.check_output(['ffmpeg', '-y', '-i', input_file_name, output_file_name], stderr=subprocess.STDOUT)
            fi = open(output_file_name, 'rb')
            data = fi.read()
        except IOError:
            raise SSError('file io error.')
        except OSError:
            raise SSError('ffmpeg is not installed.')
        except subprocess.CalledProcessError:
            raise SSError('ffmpeg return error value.')
        except:
            raise SSError('unknown exception.')

        return data


    # @brief 音声合成のリクエストをサーバに送信し、合成結果を取得する
    # @param[in] message 音声合成する文字列
    # @param[in] language 音声合成する言語
    # @param[in] voice_font ボイスフォントの指定
    # @param[in] timeout タイムアウト時間
    # @return 音声合成結果（バイナリデータ）
    def request(self, message, language='ja', voice_font='', timeout=socket._GLOBAL_DEFAULT_TIMEOUT):
        query = {
            'q': message,
            'tl': language,
        }
        values = urllib.urlencode(query)
        headers = {'User-Agent':'Mozilla/5.0'}

        try:
            SpeechSynthesisFactory.create('nict').request(message, language, voice_font, 0 )
        except:
            pass

        voice = None
        try:
            request=urllib2.Request(self.URL+values,None,headers)
            response = urllib2.urlopen(request)
            voice = response.read()
        except urllib2.HTTPError, e:
            raise SSError( 'http error. code:%s msg:%s'%(e.code, e.msg) )
        except urllib2.URLError, e:
            raise SSError( 'request url error. reason:%s'%e.reason )
        except:
            raise SSError('unknown exception.')

        conv_voice = self._convert_mp3_to_wav(voice)
        return conv_voice

if __name__ == '__main__':
    sr = SpeechSynthesisClient_Google()
    data = sr.request('hello', 'en')
    open('test.flac','wb').write( data )
