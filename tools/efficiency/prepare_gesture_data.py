#!/usr/bin/env python3
"""Cache reproducible, subject-separated HaGRID gesture development data.

Human gesture labels supervise classification; the supplied 2D landmarks are
automatic MediaPipe predictions, not independent landmark ground truth. They
are normalized by image width and height separately, and annotations do not
include image dimensions. Images and full annotations remain outside Git.
"""
import argparse
import concurrent.futures
import hashlib
import json
import zipfile
from pathlib import Path

import numpy as np

from remote_zip import RemoteZipFile

ROOT = 'https://rndml-team-cv.obs.ru-moscow-1.hc.sbercloud.ru/datasets/'
ANNOT = ROOT + 'hagrid_v2/annotations_with_landmarks/annotations.zip'
CLASSES = ['palm', 'like', 'dislike', 'one', 'point', 'fist', 'no_gesture']
TRAIN_CLASSES = ['point', 'one', 'palm', 'like', 'dislike', 'fist', 'no_gesture', 'peace', 'ok']
OLD_CLASSES = {'palm', 'like', 'dislike', 'fist', 'peace', 'ok'}
SEED = 'luxoiii-gesture-v2:'


def digest(value):
    return hashlib.sha256(value if isinstance(value, bytes) else value.encode()).hexdigest()


def rank(value):
    return digest(SEED + value)


def annotations(root, split, label, old_cache=None):
    path = root / 'annotations' / split / (label + '.json')
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        previous = old_cache / (label + '-test.json') if old_cache and split == 'test' else None
        if previous and previous.exists():
            path.write_bytes(previous.read_bytes())
        else:
            with zipfile.ZipFile(RemoteZipFile(ANNOT)) as archive:
                path.write_bytes(archive.read('annotations/' + split + '/' + label + '.json'))
    return json.loads(path.read_text()), digest(path.read_bytes())


def select(data, count, excluded=frozenset()):
    selected, seen = [], set()
    for image_id in sorted(data, key=rank):
        row = data[image_id]
        subject = digest(row['user_id'])
        if subject in seen or subject in excluded:
            continue
        seen.add(subject)
        selected.append((image_id, row))
        if len(selected) == count:
            break
    if len(selected) < count:
        print(f'Only {len(selected)} unique available subjects (requested {count})', flush=True)
    return selected


def cache_landmarks(root, split, label, count):
    data, annotation_sha = annotations(root, split, label)
    points, boxes, labels, image_ids, subject_hashes = [], [], [], [], []
    malformed = 0
    for image_id, row in select(data, count):
        landmarks = row.get('hand_landmarks', [])
        if len(landmarks) != len(row['labels']) or len(row['bboxes']) != len(row['labels']):
            malformed += 1
            continue
        for hand, box, target in zip(landmarks, row['bboxes'], row['labels']):
            array = np.asarray(hand, dtype=np.float32)
            if array.shape != (21, 2) or not np.isfinite(array).all():
                malformed += 1
                continue
            points.append(array)
            boxes.append(box)
            labels.append(target)
            image_ids.append(image_id)
            subject_hashes.append(digest(row['user_id']))
    destination = root / 'landmarks' / f'{split}-{label}.npz'
    destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(destination, points=np.asarray(points, dtype=np.float32),
                        boxes=np.asarray(boxes, dtype=np.float32), labels=np.asarray(labels),
                        image_ids=np.asarray(image_ids), subject_hashes=np.asarray(subject_hashes))
    result = dict(split=split, category=label, hands=len(points), images=len(set(image_ids)),
                  subjects=len(set(subject_hashes)), skipped_malformed=malformed,
                  path=str(destination.relative_to(root)), sha256=digest(destination.read_bytes()),
                  annotation_sha256=annotation_sha)
    print(f'{split}/{label}: {len(points)} landmark hands ready at {destination}', flush=True)
    return result


def prepare_images(root, validation_count, test_count, old_manifest, old_cache):
    old = json.loads(old_manifest.read_text())
    excluded = {row['subject_hash'] for row in old['rows']}

    def category(label):
        groups = []
        for split, count in [('val', validation_count), ('test', test_count)]:
            if not count:
                continue
            data, annotation_sha = annotations(root, split, label, old_cache)
            groups.append((split, select(data, count, excluded), annotation_sha))
        location = 'hagrid/hagrid_dataset_new_554800/hagrid_dataset/' if label in OLD_CLASSES else 'hagrid_v2/hagrid_v2_zip/'
        url = ROOT + location + label + '.zip'
        remote = RemoteZipFile(url)
        rows = []
        with zipfile.ZipFile(remote) as archive:
            by_id = {Path(info.filename).stem: info for info in archive.infolist() if info.filename.endswith('.jpg')}
            for split, selected, annotation_sha in groups:
                for image_id, annotation in selected:
                    directory = root / split / 'hagrid'
                    path = directory / 'images' / label / (image_id + '.jpg')
                    path.parent.mkdir(parents=True, exist_ok=True)
                    if not path.exists():
                        path.write_bytes(archive.read(by_id[image_id]))
                    rows.append(dict(split=split, id=image_id, path=str(path.relative_to(directory)),
                                     sha256=digest(path.read_bytes()), category=label,
                                     boxes=annotation['bboxes'], labels=annotation['labels'],
                                     subject_hash=digest(annotation['user_id']), archive=url,
                                     annotation_sha256=annotation_sha))
        print(f'{label}: {len(rows)} images ready, {remote.downloaded / 1e6:.1f} MB transferred', flush=True)
        return rows

    with concurrent.futures.ThreadPoolExecutor(7) as pool:
        rows = sum(pool.map(category, CLASSES), [])
    manifests = {}
    for split, count in [('val', validation_count), ('test', test_count)]:
        if not count:
            continue
        selected = [dict(row) for row in rows if row['split'] == split]
        for row in selected:
            row.pop('split')
        result = dict(dataset='HaGRIDv2', split=split, per_category=count,
                      selection='SHA256 luxoiii-gesture-v2, one image per subject per category; exclude all original evaluation subjects',
                      excluded_manifest_sha256=digest(old_manifest.read_bytes()), excluded_subjects=len(excluded),
                      annotation_url=ANNOT, rows=selected)
        destination = root / split / 'hagrid' / 'manifest.json'
        destination.write_text(json.dumps(result, indent=2) + '\n')
        manifests[split] = result
    if 'val' in manifests and 'test' in manifests:
        val_subjects = {row['subject_hash'] for row in manifests['val']['rows']}
        test_subjects = {row['subject_hash'] for row in manifests['test']['rows']}
        assert not val_subjects & test_subjects, 'Validation and test subjects overlap'
    return manifests


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--train-landmarks', type=int, default=0)
    parser.add_argument('--val-landmarks', type=int, default=0)
    parser.add_argument('--val-images', type=int, default=0)
    parser.add_argument('--test-images', type=int, default=0)
    parser.add_argument('--old-manifest', type=Path, default=Path('reports/efficiency/hagrid-manifest.json'))
    parser.add_argument('--old-cache', type=Path, default=Path('/tmp/luxoiii-validation/hagrid'))
    args = parser.parse_args()
    args.root.mkdir(parents=True, exist_ok=True)
    jobs = [(args.root, split, label, count) for label in TRAIN_CLASSES for split, count in [('train', args.train_landmarks), ('val', args.val_landmarks)] if count]
    if jobs:
        with concurrent.futures.ThreadPoolExecutor(3) as pool:
            outputs = list(pool.map(lambda item: cache_landmarks(*item), jobs))
        subjects = {}
        for row in outputs:
            cached = np.load(args.root / row['path'], allow_pickle=False)
            subjects.setdefault(row['split'], set()).update(cached['subject_hashes'].tolist())
        if 'train' in subjects and 'val' in subjects:
            assert not subjects['train'] & subjects['val'], 'Training and validation subjects overlap'
        metadata = dict(dataset='HaGRIDv2', annotation_url=ANNOT, selection=SEED,
                        source='https://github.com/hukenovs/hagrid', license='CC BY-SA 4.0',
                        landmark_note='Auto-annotated MediaPipe 21x2 normalized image coordinates; image aspect ratio unavailable in annotations. Human gesture labels are supervision.',
                        rows=outputs)
        (args.root / 'landmarks-manifest.json').write_text(json.dumps(metadata, indent=2) + '\n')
    if args.val_images or args.test_images:
        prepare_images(args.root, args.val_images, args.test_images, args.old_manifest, args.old_cache)


if __name__ == '__main__':
    main()
