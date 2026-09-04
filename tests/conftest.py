import sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from h2air.config import load_model_config
from h2air.pipeline import run_model

@pytest.fixture(scope='session')
def model_config():return load_model_config(ROOT/'config/baseline.json')

@pytest.fixture(scope='session')
def integrated_result(tmp_path_factory,model_config):
    out=tmp_path_factory.mktemp('baseline_integrated_run');return run_model(model_config,out,write_histories=False),out

@pytest.fixture(scope='session')
def optimized_config():return load_model_config(ROOT/'config/optimized_screening_case.json')
