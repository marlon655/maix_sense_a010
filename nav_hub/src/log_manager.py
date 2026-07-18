"""
log_manager.py

Implementa:
- DailyFileHandler: handler que grava em um arquivo por dia (Prefix_DD_MM_YYYY.log),
  não renomeia arquivos antigos e faz limpeza de arquivos mais antigos que N meses.
- LogManager: wrapper de alto nível que expõe `gera_log(mensagem, tipo_log)` e
  configura dois handlers (info/debug e warn/error/fatal).

Observações:
- A limpeza pode ser custosa se executada a cada escrita (configurável).
- Não é seguro entre múltiplos processos que escrevem simultaneamente no mesmo arquivo.
"""

from pathlib import Path
import logging
from datetime import datetime, date, timedelta
import calendar
import re
import time
from typing import Optional
import sys

# -----------------------------
# DailyFileHandler
# -----------------------------
class DailyFileHandler(logging.Handler):
    """
    Handler de logging que:
    - Grava em arquivos diários com nome "<prefix>_DD_MM_YYYY.log" (modo append).
    - Ao mudar de dia, fecha o arquivo anterior e abre o arquivo do dia atual.
    - Remove arquivos cujo nome bate com o padrão e cuja data seja anterior a
      retention_months (ex.: 3 meses).
    - Pode limitar a frequência da limpeza com cleanup_interval_seconds.
    """

    def __init__(
        self,
        directory: Path,
        prefix: str,
        retention_months: int = 3,
        level: int = logging.NOTSET,
        formatter: Optional[logging.Formatter] = None,
        cleanup_interval_seconds: int = 0,  # 0 -> executar limpeza a cada emit
    ):
        """
        Args:
            directory: Path do diretório onde serão gravados os arquivos.
            prefix: Prefixo do arquivo, ex.: "Log_Info".
            retention_months: quantos meses manter (arquivos mais antigos serão apagados).
            level: nível do handler.
            formatter: Formatter opcional (se passado, será aplicado).
            cleanup_interval_seconds: intervalo mínimo, em segundos, entre execuções de cleanup.
        """
        super().__init__(level)

        self.terminator = "\n"      # garante que exista terminador como em StreamHandler
        self.encoding = "utf-8"     # informação explícita do encoding (opcional)

        # caminho do diretório onde os logs serão gravados
        self.directory = Path(directory)
        # prefixo do arquivo diário
        self.prefix = prefix
        # retenção em meses
        self.retention_months = int(retention_months)
        # data atualmente aberta no stream (None antes da primeira escrita)
        self.current_date: Optional[date] = None
        # stream de arquivo (arquivo aberto para append)
        self.stream = None

        # controle de frequência do cleanup
        self.last_cleanup_time = 0.0
        self.cleanup_interval_seconds = int(cleanup_interval_seconds)

        # garante criação do diretório (parents=True -> cria a árvore se necessário)
        self.directory.mkdir(parents=True, exist_ok=True)

        # aplica formatter se fornecido
        if formatter:
            self.setFormatter(formatter)

        # regex para reconhecer arquivos com formato "<prefix>_DD_MM_YYYY.log"
        self._pattern = re.compile(rf"^{re.escape(self.prefix)}_(\d{{2}}_\d{{2}}_\d{{4}})\.log$")

    # ---------- helpers ----------
    @staticmethod
    def _subtract_months(d: date, months: int) -> date:
        """
        Subtrai `months` meses de `d`. Ajusta o dia para o último dia do mês alvo
        caso necessário (ex.: de 31 para 30).
        """
        if months <= 0:
            return d
        y = d.year
        m = d.month - months
        # ajusta ano/mes se m <= 0
        while m <= 0:
            m += 12
            y -= 1
        # obtém o último dia do mês alvo
        last_day = calendar.monthrange(y, m)[1]
        day = min(d.day, last_day)
        return date(y, m, day)

    def _filename_for_date(self, d: date) -> Path:
        """Retorna o Path do arquivo correspondente à data `d`."""
        return self.directory / f"{self.prefix}_{d.strftime('%d_%m_%Y')}.log"
    

    def _open_stream_for(self, d: date):
        """Abre (ou cria) o arquivo do dia em modo append e retorna o stream."""
        path = self._filename_for_date(d)
        # 'a' para não sobrescrever; encoding utf-8 para acentuação correta
        return open(path, "a", encoding="utf-8")

    def _cleanup_old_files(self):
        """
        Remove arquivos cujo nome bate com o padrão e que sejam anteriores ao cutoff.
        O cutoff é calculado subtraindo `retention_months` da data de hoje.
        """
        if not self.retention_months or self.retention_months <= 0:
            return

        today = date.today()
        cutoff = self._subtract_months(today, self.retention_months)

        # itera todos os arquivos do diretório
        for f in self.directory.iterdir():
            if not f.is_file():
                continue
            m = self._pattern.match(f.name)
            if not m:
                # ignora arquivos fora do padrão
                continue
            try:
                file_date = datetime.strptime(m.group(1), "%d_%m_%Y").date()
            except Exception:
                # se o nome não for parseável, ignora
                continue
            # se a data do arquivo for anterior ao cutoff, tenta apagar
            if file_date < cutoff:
                try:
                    f.unlink()
                except Exception:
                    # falhas ao apagar não devem interromper o logging
                    pass

    # ---------- método principal do Handler ----------
    def emit(self, record: logging.LogRecord):
        """
        Escrito para cada LogRecord:
        - garante que o stream do dia atual esteja aberto;
        - escreve a mensagem formatada e dá flush;
        - executa limpeza periódica (ou a cada emit se cleanup_interval_seconds==0).
        """
        try:
            # bloqueia o handler para garantir thread-safety
            self.acquire()

            today = date.today()

            # Se o dia mudou (ou primeira vez), troca de arquivo
            if self.current_date != today:
                # fecha stream anterior se existir
                if self.stream:
                    try:
                        self.stream.close()
                    except Exception:
                        pass
                # tenta abrir o novo arquivo do dia atual
                try:
                    self.stream = self._open_stream_for(today)
                    self.current_date = today
                except Exception as e:
                    # se não conseguir abrir, marca stream como None e aborta escrita
                    print(f"[DailyFileHandler ERROR] Não abriu arquivo: {e}")
                    self.stream = None

            if not self.stream:
                # aborta gravação se não houver arquivo disponível
                print(f"[DailyFileHandler] ERRO: não conseguiu abrir arquivo para {self.prefix}")
                return

            # decide se roda o cleanup agora (conforme intervalo configurado)
            now_ts = time.time()
            run_cleanup = False
            if self.cleanup_interval_seconds <= 0:
                # 0 => executar a cada escrita (comportamento solicitado)
                run_cleanup = True
            else:
                if (now_ts - self.last_cleanup_time) >= self.cleanup_interval_seconds:
                    run_cleanup = True

            if run_cleanup:
                try:
                    self._cleanup_old_files()
                except Exception:
                    # nunca propagar erro de limpeza
                    pass
                self.last_cleanup_time = now_ts

            # formata a mensagem (usa formatter do handler, se configurado)
            msg = self.format(record)
            # escreve + terminator (normalmente '\n') e garante flush
            self.stream.write(msg + self.terminator)
            self.stream.flush()

        except Exception:
            # handleError segue a política do logging (registra exceção internamente)
            self.handleError(record)
        finally:
            # libera o lock sempre
            self.release()

    def close(self):
        """Fecha o stream aberto e executa o fechamento padrão do Handler."""
        try:
            if self.stream:
                try:
                    self.stream.close()
                except Exception:
                    pass
                self.stream = None
        finally:
            super().close()


# -----------------------------
# LogManager
# -----------------------------
class LogManager:
    """
    Gerenciador de logs de alto nível que:
    - cria um Logger por instância;
    - cria dois DailyFileHandler (info/debug e warning+);
    - expõe a API compatível `gera_log(mensagem, tipo_log)`.
    """

    # constantes de nível compatíveis com o seu código atual
    Fatal = "Fatal"
    Error = "Error"
    Warn  = "Warn"
    Info  = "Info"
    Debug = "Debug"

    def __init__(
        self,
        base_dir: str = "/home/ubuntu/Desktop/logs",
        info_prefix: str = "Log_Info",
        error_prefix: str = "Log_Error",
        retention_months: int = 3,
        cleanup_interval_seconds: int = 0,
        console: bool = False,
        console_level=logging.INFO
    ):
        """
        Inicializa o LogManager.

        Args:
            base_dir: diretório base onde serão criadas as pastas 'info' e 'erro'.
            info_prefix / error_prefix: prefixos dos arquivos diários.
            retention_months: quantos meses manter (ex.: 3).
            cleanup_interval_seconds: intervalo mínimo entre limpezas (0 = sempre).
        """
        # cria os diretórios necessários
        self.base_dir = Path(base_dir)
        self.info_dir = self.base_dir / "info"
        self.error_dir = self.base_dir / "error"
        self.info_dir.mkdir(parents=True, exist_ok=True)
        self.error_dir.mkdir(parents=True, exist_ok=True)

        # cria um logger único por instância (evita colidir com outros loggers)
        self._logger = logging.getLogger(f"LogManager_{id(self)}")
        self._logger.setLevel(logging.DEBUG)  # captura todas as mensagens; handlers filtram depois

        # define o formatter para as linhas de log:
        # exemplo: "16_10_2025 14:30:01 ; [INFO] ; Robô em movimento."
        fmt = "%(asctime)s ; [%(levelname)s] ; %(message)s"
        datefmt = "%d-%m-%Y %H:%M:%S"
        formatter = logging.Formatter(fmt, datefmt=datefmt)

        # instancia os handlers diários:
        # - info_handler grava DEBUG/INFO (level=logging.DEBUG)
        # - error_handler grava WARNING/ERROR/CRITICAL (level=logging.WARNING)
        self.info_handler = DailyFileHandler(
            directory=self.info_dir,
            prefix=info_prefix,
            retention_months=retention_months,
            level=logging.DEBUG,
            formatter=formatter,
            cleanup_interval_seconds=cleanup_interval_seconds,
        )

        self.error_handler = DailyFileHandler(
            directory=self.error_dir,
            prefix=error_prefix,
            retention_months=retention_months,
            level=logging.WARNING,
            formatter=formatter,
            cleanup_interval_seconds=cleanup_interval_seconds,
        )

        # adiciona handlers ao logger somente se ainda não tiver handlers
        # (evita adicionar múltiplos handlers caso a mesma instância seja recriada)
        # if not self._logger.handlers:
        #     self._logger.addHandler(self.info_handler)
        #     self._logger.addHandler(self.error_handler)

        #     if console:
        #         stream_handler = logging.StreamHandler(stream=sys.stdout)
        #         stream_handler.setLevel(console_level)
        #         stream_handler.setFormatter(formatter)
        #         self._logger.addHandler(stream_handler)

        #     # evita que as mensagens sejam encaminhadas ao logger root (duplicações)
        #     self._logger.propagate = False

        # força a adição dos handlers, mesmo que o logger já exista
        self._logger.addHandler(self.info_handler)
        self._logger.addHandler(self.error_handler)

        if console:
            stream_handler = logging.StreamHandler(stream=sys.stdout)
            stream_handler.setLevel(console_level)
            stream_handler.setFormatter(formatter)
            self._logger.addHandler(stream_handler)

        self._logger.propagate = False

        # limpeza inicial: remove arquivos antigos já existentes (tenta, não falha)
        try:
            self.info_handler._cleanup_old_files()
            self.error_handler._cleanup_old_files()
        except Exception:
            pass

    def gera_log(self, mensagem: str, tipo_log: str):
        """
        API pública compatível com seu uso atual:
            gera_log("mensagem", LogManager.Info)

        Converte tipo_log para string e mapeia para os métodos do logging.
        """
        tipo_log = str(tipo_log)

        if tipo_log == self.Debug:
            self._logger.debug(mensagem)
        elif tipo_log == self.Info:
            self._logger.info(mensagem)
        elif tipo_log == self.Warn:
            self._logger.warning(mensagem)
        elif tipo_log == self.Error:
            self._logger.error(mensagem)
        elif tipo_log == self.Fatal:
            # Fatal mapeado para CRITICAL no logging
            self._logger.critical(mensagem)
        else:
            # nível desconhecido -> grava como INFO e marca nível desconhecido
            self._logger.info(f"[UNKNOWN LEVEL: {tipo_log}] {mensagem}")
