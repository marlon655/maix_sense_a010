#!/usr/bin/env python3
import json
import sys
import argparse

def convert_geojson_to_goals(input_file, ids_to_extract):
    with open(input_file, 'r') as f:
        geojson_data = json.load(f)
    
    default_covariance = [
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
    ]
    
    features_map = {}
    for feature in geojson_data.get('features', []):
        if feature.get('geometry', {}).get('type') != 'Point':
            continue
            
        feature_id = feature.get('properties', {}).get('id')
        if feature_id in ids_to_extract:
            coords = feature.get('geometry', {}).get('coordinates', [])
            if len(coords) >= 2:
                features_map[feature_id] = {
                    'x': coords[0],
                    'y': coords[1]
                }
    
    ordered_features = []
    for feature_id in ids_to_extract:
        if feature_id in features_map:
            ordered_features.append(features_map[feature_id])
        else:
            print(f"Warning: ID {feature_id} wasnt found in geojson", file=sys.stderr)
    
    if not ordered_features:
        print("Error: no valid IDs found in geojson", file=sys.stderr)
        sys.exit(1)
    
    goals = {}
    
    first_point = ordered_features[0]
    goals['initial'] = {
        'sequence': 1,
        'covariance': default_covariance,
        'w': 0.0,
        'y': first_point['y'],
        'x': first_point['x'],
        'z': 0.0
    }
    
    sequence = 2
    for i, point in enumerate(ordered_features):
        if i == len(ordered_features)-1:
            break
        step_name = f'step{i}'
        goals[step_name] = {
            'sequence': sequence,
            'w': 0.0,
            'y': point['y'],
            'x': point['x'],
            'z': 0.0
        }
        sequence += 1
    
    last_point = ordered_features[-1]
    goals['ending'] = {
        'sequence': sequence,
        'w': 0.0,
        'y': last_point['y'],
        'x': last_point['x'],
        'z': 0.0
    }
    
    with open('goals.json', 'w') as f:
        json.dump(goals, f, indent=4)
    
    print(f"goals.json created successfully ({len(ordered_features)} goals)")

def parse_ids(ids_string):
    # accept both comma-separated and space-separated formats
    if ',' in ids_string[0]:
        ids = [int(x.strip().split(',')[0]) for x in ids_string if x.strip()]
    else:
        ids = [int(x.strip()) for x in ids_string if x.strip()]
    return ids

def main():
    parser = argparse.ArgumentParser(
        description='Convert Nav2 GeoJSON to goals.json format'
    )
    parser.add_argument(
        'input_file',
        help='Input GeoJSON file (relative or absolute path)'
    )
    parser.add_argument(
        'ids',
        nargs= '+', 
        help='IDs separated by comma or space (e.g., "1,2,3" or "1, 2, 3" or "1 2 3")'
    )
    
    args = parser.parse_args()
    
    ids_to_extract = parse_ids(args.ids)
    
    convert_geojson_to_goals(args.input_file, ids_to_extract)

main()