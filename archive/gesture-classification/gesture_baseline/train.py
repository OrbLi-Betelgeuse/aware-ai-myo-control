"""Train RMS + shrinkage LDA; evaluate by holding out whole recording files."""
from __future__ import annotations
import argparse, csv, json, platform
from collections import Counter
from pathlib import Path
import joblib
import numpy as np
import sklearn
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import classification_report, confusion_matrix, balanced_accuracy_score, accuracy_score, f1_score
from sklearn.model_selection import StratifiedKFold
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from gesture_model import CHANNELS, load_dataset

def classifier(labels):
    return make_pipeline(StandardScaler(), LinearDiscriminantAnalysis(
        solver='lsqr', shrinkage='auto', priors=np.ones(len(labels))/len(labels)))

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data', required=True, help='ZIP or directory with class folders')
    ap.add_argument('--config', default=str(Path(__file__).with_name('config.json')))
    ap.add_argument('--out', default=str(Path(__file__).with_name('outputs')))
    args = ap.parse_args()
    config = json.loads(Path(args.config).read_text())
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    X,y,groups,audit = load_dataset(args.data, config)
    labels = sorted(config['labels'])
    file_names = np.array([r['file'] for r in audit])
    file_labels = np.array([r['label'] for r in audit])
    folds = min(config['folds'], min(Counter(file_labels).values()))
    if folds < 2:
        raise ValueError('Need at least two independent recording files per class for evaluation.')
    cv = StratifiedKFold(folds, shuffle=True, random_state=config['seed'])
    predicted = np.empty(len(y), dtype=object)
    fold_for = np.zeros(len(y), dtype=int)
    results, assignments = [], []
    for fold,(train,test) in enumerate(cv.split(file_names,file_labels),1):
        train_mask = np.isin(groups,file_names[train]); test_mask=np.isin(groups,file_names[test])
        assert not set(groups[train_mask]) & set(groups[test_mask])
        model = classifier(labels).fit(X[train_mask],y[train_mask])
        p = model.predict(X[test_mask]); predicted[test_mask]=p; fold_for[test_mask]=fold
        results.append(dict(fold=fold, training_files=len(train), test_files=len(test),
            accuracy=float(accuracy_score(y[test_mask],p)),
            balanced_accuracy=float(balanced_accuracy_score(y[test_mask],p)),
            macro_f1=float(f1_score(y[test_mask],p,average='macro',zero_division=0))))
        assignments.extend(dict(file=file_names[i],label=file_labels[i],test_fold=fold) for i in test)
    # Final deployable artifact uses all supplied recordings, after out-of-fold evaluation.
    model = classifier(labels).fit(X,y)
    joblib.dump(dict(model=model, config=config, channels=CHANNELS, classes=model.classes_.tolist(),
                     feature='raw_rms_per_channel', sklearn_version=sklearn.__version__),out/'model.joblib')
    cm=confusion_matrix(y,predicted,labels=labels)
    metrics=dict(evaluation='Stratified recording-file cross-validation; NOT cross-user validation',
        classes=labels, recording_counts=dict(Counter(file_labels)),window_counts=dict(Counter(y)),
        n_recordings=len(audit),n_windows=len(y),folds=results,
        out_of_fold_accuracy=float(accuracy_score(y,predicted)),
        out_of_fold_balanced_accuracy=float(balanced_accuracy_score(y,predicted)),
        out_of_fold_macro_f1=float(f1_score(y,predicted,average='macro',zero_division=0)),
        classification_report=classification_report(y,predicted,labels=labels,output_dict=True,zero_division=0),
        confusion_matrix=cm.tolist(),seed=config['seed'],config=config,
        versions=dict(python=platform.python_version(),numpy=np.__version__,sklearn=sklearn.__version__),
        limitations=[
            'No rest/neutral examples: predictions cannot validate stopping or idle false activations.',
            'All current recordings are from Shima (user-confirmed): within-person evaluation only.',
            'One CSV is assumed to be one independent trial with its folder label throughout.',
            'Classes were collected in time blocks; session/time/placement effects may confound results.',
            'Timestamps are nonmonotonic; preserve row order and use nominal-rate sample-count windows.',
            'Variable acquisition rates mean 40 samples may span more than 200 ms.',
            'Overlapping windows are correlated: window counts are not independent trial counts.',
            'Final model uses all data; evaluate future new trials separately before retraining.',
            'Probability is not calibrated and is not a validated unknown-gesture detector.'
        ])
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2))
    (out/'data_audit.json').write_text(json.dumps(audit,indent=2))
    for filename, rows in [('fold_assignments.csv',assignments),('recording_audit.csv',audit)]:
        with (out/filename).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    with (out/'confusion_matrix.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['actual / predicted',*labels]);w.writerows([label,*row] for label,row in zip(labels,cm))
    with (out/'oof_predictions.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['recording','window_index_in_dataset','test_fold','actual','predicted'])
        w.writerows(zip(groups,range(len(y)),fold_for,y,predicted))
    text=[f'Recordings: {len(audit)}; windows: {len(y)}',f'Recordings by class: {{str(k):v for k,v in Counter(file_labels).items()}}',
          f'{folds}-fold stratified validation, split by whole recording files.',
          f'OOF accuracy: {metrics["out_of_fold_accuracy"]:.3f}',
          f'OOF balanced accuracy: {metrics["out_of_fold_balanced_accuracy"]:.3f}',
          f'OOF macro F1: {metrics["out_of_fold_macro_f1"]:.3f}',
          '',classification_report(y,predicted,labels=labels,zero_division=0),'LIMITATIONS:',*metrics['limitations']]
    (out/'RESULTS.txt').write_text('\n'.join(text))
    print('\n'.join(text))
    # Verify round trip and class order after serialization.
    restored=joblib.load(out/'model.joblib')
    np.testing.assert_array_equal(restored['model'].predict(X[:10]),model.predict(X[:10]))
    print('Saved model reload verified:',out/'model.joblib')

if __name__=='__main__': main()
