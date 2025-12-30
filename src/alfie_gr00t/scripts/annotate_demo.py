#!/usr/bin/env python3
"""
Demonstration Annotation Tool

After recording, use this to label demonstrations as successful/failed
and add quality annotations for training dataset curation.
"""

import os
import json
import argparse
from pathlib import Path
from datetime import datetime


class DemoAnnotator:
    """Annotate recorded demonstrations with labels and metadata"""

    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)
        self.manifest_file = self.data_dir / 'manifest.json'
        self.manifest = self.load_manifest()

    def load_manifest(self):
        """Load or create manifest file"""
        if self.manifest_file.exists():
            with open(self.manifest_file, 'r') as f:
                return json.load(f)
        else:
            return {
                'created_at': datetime.now().isoformat(),
                'demonstrations': []
            }

    def save_manifest(self):
        """Save manifest to file"""
        with open(self.manifest_file, 'w') as f:
            json.dump(self.manifest, indent=2, fp=f)
        print(f"✓ Manifest saved: {self.manifest_file}")

    def list_demonstrations(self):
        """List all recorded demonstrations"""
        print("\n" + "="*60)
        print("Recorded Demonstrations")
        print("="*60)

        # Find all bag directories
        bag_dirs = sorted([d for d in self.data_dir.iterdir() if d.is_dir() and d.name.startswith('demo_')])

        if not bag_dirs:
            print("No demonstrations found!")
            return []

        demos = []
        for i, bag_dir in enumerate(bag_dirs, 1):
            # Check if already annotated
            existing = next(
                (d for d in self.manifest['demonstrations'] if d['bag_name'] == bag_dir.name),
                None
            )

            status = "✓ Annotated" if existing else "○ Not annotated"
            success = existing.get('success', 'unknown') if existing else 'unknown'

            print(f"{i:3d}. {bag_dir.name:30s} [{status}] Success: {success}")
            demos.append({
                'index': i,
                'path': bag_dir,
                'name': bag_dir.name,
                'existing': existing
            })

        print("="*60)
        return demos

    def annotate_demo(self, demo_path, demo_name):
        """Interactively annotate a demonstration"""
        print(f"\nAnnotating: {demo_name}")
        print("-" * 60)

        # Check if already annotated
        existing = next(
            (d for d in self.manifest['demonstrations'] if d['bag_name'] == demo_name),
            None
        )

        if existing:
            print(f"Existing annotation found:")
            print(f"  Success: {existing['success']}")
            print(f"  Quality: {existing.get('quality', 'N/A')}")
            print(f"  Notes:   {existing.get('notes', 'N/A')}")
            print()
            overwrite = input("Overwrite? (y/n): ").lower().strip()
            if overwrite != 'y':
                return

        # Collect annotations
        print("\nWas this demonstration successful? (y/n/partial): ", end='')
        success = input().lower().strip()

        success_map = {'y': True, 'n': False, 'partial': 'partial'}
        success_value = success_map.get(success, False)

        print("\nGrasp quality (1-5, where 5 is best): ", end='')
        quality_str = input().strip()
        quality = int(quality_str) if quality_str.isdigit() else None

        print("\nCan position (floor/table/elevated/other): ", end='')
        can_position = input().strip() or 'unknown'

        print("\nLighting condition (bright/normal/dim): ", end='')
        lighting = input().strip() or 'normal'

        print("\nNotes (optional): ", end='')
        notes = input().strip()

        # Create annotation
        annotation = {
            'bag_name': demo_name,
            'bag_path': str(demo_path),
            'success': success_value,
            'quality': quality,
            'can_position': can_position,
            'lighting': lighting,
            'notes': notes,
            'annotated_at': datetime.now().isoformat()
        }

        # Remove existing and add new
        if existing:
            self.manifest['demonstrations'] = [
                d for d in self.manifest['demonstrations']
                if d['bag_name'] != demo_name
            ]

        self.manifest['demonstrations'].append(annotation)
        print(f"\n✓ Annotation added for {demo_name}")

    def show_statistics(self):
        """Show dataset statistics"""
        demos = self.manifest['demonstrations']

        if not demos:
            print("\nNo annotated demonstrations yet!")
            return

        total = len(demos)
        successful = sum(1 for d in demos if d['success'] is True)
        failed = sum(1 for d in demos if d['success'] is False)
        partial = sum(1 for d in demos if d['success'] == 'partial')

        avg_quality = sum(d['quality'] for d in demos if d.get('quality')) / total

        print("\n" + "="*60)
        print("Dataset Statistics")
        print("="*60)
        print(f"Total demonstrations:    {total}")
        print(f"  Successful:            {successful} ({successful/total*100:.1f}%)")
        print(f"  Partial success:       {partial} ({partial/total*100:.1f}%)")
        print(f"  Failed:                {failed} ({failed/total*100:.1f}%)")
        print(f"Average quality:         {avg_quality:.2f}/5.0")
        print("="*60)

    def interactive_session(self):
        """Run interactive annotation session"""
        while True:
            demos = self.list_demonstrations()

            if not demos:
                break

            print("\nOptions:")
            print("  [1-N]  - Annotate demonstration N")
            print("  [s]    - Show statistics")
            print("  [q]    - Quit and save")
            print()
            choice = input("Select: ").strip().lower()

            if choice == 'q':
                self.save_manifest()
                break
            elif choice == 's':
                self.show_statistics()
            elif choice.isdigit():
                idx = int(choice)
                if 1 <= idx <= len(demos):
                    demo = demos[idx - 1]
                    self.annotate_demo(demo['path'], demo['name'])
                    self.save_manifest()
                else:
                    print(f"Invalid index: {idx}")
            else:
                print(f"Unknown option: {choice}")

        print("\n✓ Annotation session complete!")


def main():
    parser = argparse.ArgumentParser(
        description='Annotate GR00T demonstration recordings'
    )
    parser.add_argument(
        '--data-dir',
        default=os.path.expanduser('~/alfiebot_ws/data/demonstrations'),
        help='Directory containing demonstration recordings'
    )
    parser.add_argument(
        '--stats',
        action='store_true',
        help='Show statistics only'
    )

    args = parser.parse_args()

    annotator = DemoAnnotator(args.data_dir)

    if args.stats:
        annotator.show_statistics()
    else:
        annotator.interactive_session()


if __name__ == '__main__':
    main()
